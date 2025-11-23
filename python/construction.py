import numpy as np
import numpy.typing as npt
from structures import *
from pathlib import Path


def calc_my_metric(I: Instance, a: float) -> npt.NDArray:
    n = I.n

    # pickup and drop node indices
    pickup = np.arange(1, 1 + n)
    drop = np.arange(1 + n, 1 + 2 * n)

    # solo trip distance for each request
    solo = (
        I.dist[0, pickup]  # depot -> pickup
        + I.dist[pickup, drop]  # pickup -> drop
        + I.dist[drop, 0]  # drop -> depot
    )

    # proper normalizers
    max_dist = np.max(solo)
    max_dem = np.max(I.demands)

    # avoid division by zero (degenerate case)
    if max_dist == 0:
        max_dist = 1
    if max_dem == 0:
        max_dem = 1

    # weighted cost metric
    costs = a * (solo / max_dist) + (1 - a) * (I.demands / max_dem)
    return costs


def construction(
    I: Instance, a: float, sigma_factor: float = 0.1, is_random: bool = False
) -> Solution:
    costs = calc_my_metric(I, a)

    if is_random:
        sigma = sigma_factor * max(costs.std() if costs.std() > 0 else 1.0, 1e-6)
        noisy_costs = costs + np.random.normal(0, sigma, size=I.n)
    else:
        noisy_costs = costs

    perm = np.argsort(noisy_costs)
    important = perm[: I.gamma]

    per_track_requests = [important[i :: I.nK] for i in range(I.nK)]

    routes = []
    for track in range(I.nK):
        route = []
        cargo = 0
        active = []

        for req in per_track_requests[track]:
            pickup = 1 + req
            dem = I.demands[req]

            # capacity check: drop heaviest if needed
            if cargo + dem > I.C:
                if not active:
                    continue
                heaviest = min(active, key=lambda r: I.demands[r])
                active.remove(heaviest)
                route.append(1 + I.n + heaviest)
                cargo -= I.demands[heaviest]

            # pick this request
            route.append(pickup)
            active.append(req)
            cargo += dem

        # drop remaining active requests
        active.sort(key=lambda r: I.demands[r], reverse=True)
        for r in active:
            route.append(1 + I.n + r)

        routes.append(route)

    # In case we refuse some roots (???)
    served = set()
    for r in routes:
        for node in r:
            req = I.request_of_node[node]
            if req >= 0:
                served.add(req)

    if len(served) < I.gamma:
        missing = [req for req in perm if req not in served]
        # Add simplest fix: append pickup+drop to the shortest route (if feasible)
        for req in missing:
            if len(served) >= I.gamma:
                break
            # try to insert missing req at end of some route
            pickup = 1 + req
            drop = 1 + I.n + req
            for k in range(I.nK):
                r = routes[k]
                new_r = r + [pickup, drop]
                cargo = calc_route_cargo(I, new_r)
                if np.all((0 <= cargo) & (cargo <= I.C)):
                    routes[k] = new_r
                    served.add(req)
                    break

    return Solution(routes=routes)


def beam_search(I: Instance, a: float, beam_width: int = 5) -> Solution:
    costs = calc_my_metric(I, a)

    perm = np.argsort(costs)
    gamma = min(I.gamma, I.n)
    important = perm[:gamma]
    per_track_requests = [important[i :: I.nK] for i in range(I.nK)]

    n = I.n
    routes = []

    for track in range(I.nK):

        # Each element: (score, route, cargo, active_set, remaining_requests)
        remaining = list(per_track_requests[track])
        partial_routes = [(0, [], 0, frozenset(), tuple(remaining))]

        for _ in range(len(remaining)):

            new_beam = []

            for score, route, cargo, active, rem in partial_routes:
                rem = list(rem)

                # ---- 1. Try picking any request still remaining ----
                for req in rem:
                    dem = I.demands[req]
                    if cargo + dem <= I.C:
                        p = 1 + req
                        new_route = route + [p]
                        new_cargo = cargo + dem
                        new_active = active | {req}
                        new_remaining = tuple(r for r in rem if r != req)
                        last = route[-1] if route else 0
                        new_score = score + I.dist[last, p]

                        # new_score = score + I.dist[p][0]  # crude g-score, improve later
                        new_beam.append(
                            (new_score, new_route, new_cargo, new_active, new_remaining)
                        )

                # ---- 2. Try dropping any active request ----
                for req in active:
                    d = 1 + n + req
                    new_route = route + [d]
                    new_cargo = cargo - I.demands[req]
                    new_active = frozenset(r for r in active if r != req)
                    new_remaining = tuple(rem)
                    last = route[-1] if route else 0
                    new_score = score + I.dist[last, d]   # FIXED


                    # new_score = score + I.dist[d][0]
                    new_beam.append(
                        (new_score, new_route, new_cargo, new_active, new_remaining)
                    )

            # prune beam
            new_beam.sort(key=lambda x: x[0])
            partial_routes = new_beam[:beam_width]

        # finalise each candidate by dropping all active requests
        best_score = float("inf")
        best_route = None

        for score, route, cargo, active, rem in partial_routes:
            final_route = list(route)
            # drop everything still active
            last = final_route[-1] if final_route else 0
            for req in sorted(active, key=lambda r: I.dist[last, 1 + n + r]):
                final_route.append(1 + n + req)

            # evaluate full route length
            dist = route_distance(I, final_route)
            if dist < best_score:
                best_score = dist
                best_route = final_route

        routes.append(best_route)

    return Solution(routes=routes)



