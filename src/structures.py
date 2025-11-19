from dataclasses import dataclass
from abc import ABC
from copy import copy, deepcopy
import numpy as np
from numpy.typing import NDArray
import math
from typing import Iterator, List, Union, Tuple, Set, Dict
from pathlib import Path


@dataclass
class Instance:
    name: str
    n: int  # number of requests
    nK: int  # number of vehicles
    C: int  # vehicle capacity
    gamma: int  # min number of served requests
    rho: float  # fairness weight

    demands: NDArray  # shape (n,), demand per request i (1..n) at i-1
    coords: NDArray  # shape (1 + 2n, 2), coords[node_idx] = (x, y)
    dist: NDArray  # shape (1 + 2n, 1 + 2n), int travel distances

    request_of_node: NDArray  # shape (1+2n,) -> request index [0..n-1] or -1
    load_change: NDArray  # shape (1+2n,), +c_i at pickup, -c_i at drop


Route = List[int]  # list of location indices in [1, 2n], depot implicit


@dataclass
class Solution:
    routes: List[Route]  # length = nK, allow empty routes

    def write_solution(self, path: Union[str, Path], instance_name: str) -> None:
        path = Path(path)

        with path.open("w") as f:
            f.write(instance_name + "\n")
            for route in self.routes:
                if not route:
                    f.write("\n")
                else:
                    f.write(" ".join(str(node) for node in route) + "\n")



def calc_route_cargo(I: Instance, route: Route) -> NDArray:
    """
    Return an array cargo[t] = cargo *after leaving* route[t].

    The depot is implicit and has cargo = 0 at start.
    """
    cargo = 0
    out = np.zeros(len(route), dtype=int)

    for t, node in enumerate(route):
        cargo += I.load_change[node]  # pickup = +d_i, delivery = -d_i
        out[t] = cargo

    return out


def route_distance(inst: Instance, route: Route) -> int:
    if not route:
        return 0
    d = inst.dist[0, route[0]]  # depot -> first
    for u, v in zip(route, route[1:]):
        d += inst.dist[u, v]
    d += inst.dist[route[-1], 0]  # last -> depot
    return d


def all_route_distances(inst: Instance, sol: Solution) -> np.ndarray:
    return np.array([route_distance(inst, r) for r in sol.routes], dtype=float)


def jain_fairness(dists: NDArray) -> float:
    if len(dists) == 0:
        raise RuntimeError("dist has length 0!!")
    num = dists.sum() ** 2
    den = len(dists) * np.square(dists).sum()
    if den == 0:
        raise RuntimeError("Division with zero during calc of jaion fairness")
    return num / den


def objective(inst: Instance, sol: Solution) -> float:
    dists = all_route_distances(inst, sol)
    return dists.sum() + inst.rho * (1.0 - jain_fairness(dists))


def check_route_feasible(inst: Instance, route: Route) -> bool:
    load = 0
    picked = set()
    dropped = set()

    for node in route:
        req = inst.request_of_node[node]
        if req < 0:
            return False  # illegal node (e.g., depot inside route)

        load += inst.load_change[node]
        if load > inst.C or load < 0:
            return False  # capacity violation

        if inst.load_change[node] > 0:
            # pickup
            picked.add(req)
        else:
            # drop
            if req not in picked:
                return False  # drop before pickup
            dropped.add(req)

    # served requests = number of completed drop-offs
    if len(dropped) < inst.gamma:
        return False

    return True


def parse_instance(path: Path) -> Instance:
    name = path.stem

    # Read all lines exactly as they appear (keep empty lines)
    with open(path, "r") as f:
        lines = [line.rstrip("\n") for line in f]

    # Header line
    h = lines[0].split()
    n = int(h[0])
    nK = int(h[1])
    C = int(h[2])
    gamma = int(h[3])
    rho = float(h[4])

    # Find marker positions
    idx_dem = None
    idx_loc = None
    for i, line in enumerate(lines):
        if line.startswith("# demands"):
            idx_dem = i
        elif line.startswith("# request locations"):
            idx_loc = i

    if idx_dem is None or idx_loc is None:
        raise ValueError("Instance file missing required markers.")

    # Parse demands between the two markers
    demand_section = lines[idx_dem + 1 : idx_loc]
    demand_tokens = " ".join(demand_section).split()

    if len(demand_tokens) != n:
        raise ValueError(f"Expected {n} demands, found {len(demand_tokens)}.")

    demands = np.array([int(x) for x in demand_tokens], dtype=int)

    # Parse coordinates (depot + pickups + drop-offs)
    expected = 1 + 2 * n
    loc_lines = lines[idx_loc + 1 : idx_loc + 1 + expected]

    if len(loc_lines) != expected:
        raise ValueError("Incorrect number of location lines.")

    coords = np.zeros((expected, 2), dtype=float)
    for i, line in enumerate(loc_lines):
        x_str, y_str = line.split()
        coords[i] = float(x_str), float(y_str)

    # Build distance matrix (ceil Euclidean)
    nV = expected
    dist = np.zeros((nV, nV), dtype=int)
    for u in range(nV):
        for v in range(nV):
            if u != v:
                dist[u, v] = math.ceil(math.dist(coords[u], coords[v]))

    # Helper arrays: request mapping + load change
    request_of_node = np.full(nV, -1, dtype=int)
    load_change = np.zeros(nV, dtype=int)

    for i in range(n):  # request index 0..n-1
        pickup = 1 + i
        drop = 1 + n + i

        request_of_node[pickup] = i
        request_of_node[drop] = i

        c = demands[i]
        load_change[pickup] = +c
        load_change[drop] = -c

    return Instance(
        name=name,
        n=n,
        nK=nK,
        C=C,
        gamma=gamma,
        rho=rho,
        demands=demands,
        coords=coords,
        dist=dist,
        request_of_node=request_of_node,
        load_change=load_change,
    )


def parse_solution(path: str, inst: Instance) -> Solution:
    with open(path, "r") as f:
        lines = [line.rstrip("\n") for line in f]

    if not lines:
        raise ValueError("Solution file is empty")

    route_lines = lines[1:]

    routes: list[Route] = []
    for line in route_lines:
        if line == "":
            # Empty route
            routes.append([])
        else:
            nodes = [int(tok) for tok in line.split()]
            routes.append(nodes)

    # Normalize number of routes to nK
    if len(routes) < inst.nK:
        # pad with empty routes
        routes += [[] for _ in range(inst.nK - len(routes))]
    elif len(routes) > inst.nK:
        # trim extra routes
        routes = routes[: inst.nK]

    return Solution(routes=routes)
