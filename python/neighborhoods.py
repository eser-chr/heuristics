from structures import *


@dataclass
class Move:
    data: Tuple


class Neighborhood(ABC):
    def __init__(self, I: Instance, sol: Solution):
        self.I = I
        self.sol = sol
        self.f = objective(self.I, self.sol)

    def generate(self) -> Iterator[Move]:
        raise RuntimeError("Call to abstract Class")

    def apply(self, mov: Move) -> Solution:
        raise RuntimeError("Call to abstract Class")

    def is_valid(self, mov: Move) -> bool:
        raise RuntimeError("Call to abstract Class")

    def calc_delta(self, mov: Move) -> float:
        raise RuntimeError("Call to abstract Class")


class IntraRouteNeighborhood(Neighborhood):
    """
    Apply permutation inside a single tracks route.
    """

    def generate(self) -> Iterator[Move]:
        for i, route in enumerate(self.sol.routes):
            m = len(route)
            for k in range(m):
                for l in range(k + 1, m):
                    
                    # The move is the index of the 
                    # route the first node and the second node
                    yield Move(data=(i, k, l))

    def apply(self, mov: Move) -> Solution:
        i, k, l = mov.data
        new_sol = deepcopy(self.sol)
        route = new_sol.routes[i]
        route[k], route[l] = route[l], route[k]

        return new_sol

    def is_valid(self, mov: Move) -> bool:
        i, k, l = mov.data
        route = copy(self.sol.routes[i])
        route[k], route[l] = route[l], route[k]
        cargo = calc_route_cargo(self.I, route)
        return bool(np.all(cargo < self.I.C))

    
    def calc_delta(self, mov: Move) -> float:
        r, k, l = mov.data
        route = self.sol.routes[r]
        I = self.I
        dist = I.dist

        # nodes before and after positions
        x = route[k]
        y = route[l]

        # A, B around x
        A = route[k - 1] if k > 0 else 0
        B = route[k + 1] if k + 1 < len(route) else 0

        # C, D around y
        C = route[l - 1] if l > 0 else 0
        D = route[l + 1] if l + 1 < len(route) else 0

        # special case: if swapping adjacent nodes k+1 = l
        # adjust arcs (A, x, y, D)
        if l == k + 1:
            # original: A → x → y → D
            # new:      A → y → x → D
            delta = (
                dist[A, y] + dist[y, x] + dist[x, D]
                - dist[A, x] - dist[x, y] - dist[y, D]
            )
            return delta

        # general case
        delta = (
            dist[A, y] + dist[y, B] + dist[C, x] + dist[x, D]
            - dist[A, x] - dist[x, B] - dist[C, y] - dist[y, D]
        )

        return delta



# ------------------------------------------------------------
# Helper: find pickup & delivery positions of requests in route
# ------------------------------------------------------------
def pickup_delivery_positions(I: Instance, route: Route):
    """
    Returns a list of (p_idx, d_idx, req_id, pickup_node, delivery_node)
    for all requests whose pickup & delivery are in this route.
    """
    pos = {}
    for idx, node in enumerate(route):
        req = I.request_of_node[node]
        if req >= 0:
            if req not in pos:
                pos[req] = [None, None, node]  # p_idx, d_idx, pickup_or_delivery_node
            # Check whether node is pickup or delivery
            if I.load_change[node] > 0:   # pickup
                pos[req][0] = idx
            else:                          # delivery
                pos[req][1] = idx

    out = []
    for req, (p_idx, d_idx, _) in pos.items():
        if p_idx is not None and d_idx is not None:
            pickup_node = 2 * req + 1
            delivery_node = 2 * req + 2
            out.append((p_idx, d_idx, req, pickup_node, delivery_node))
    return out


# =====================================================================
# 1. Pair-Relocate Neighborhood (inter-route)
# =====================================================================

class PairRelocateNeighborhood(Neighborhood):

    def generate(self) -> Iterator[Move]:
        """
        Moves one request (pickup+delivery) from route A to route B.
        """
        for r_from, routeA in enumerate(self.sol.routes):
            # find all pickup/delivery pairs
            for (p_old, d_old, req, pnode, dnode) in pickup_delivery_positions(self.I, routeA):

                for r_to, routeB in enumerate(self.sol.routes):
                    if r_to == r_from:
                        continue

                    # Try all possible new insertion positions such that p_new < d_new
                    for p_new in range(len(routeB) + 1):
                        for d_new in range(p_new + 1, len(routeB) + 2):
                            data = ("pair_relocate",
                                    r_from, p_old, d_old,
                                    r_to, p_new, d_new,
                                    req, pnode, dnode)
                            yield Move(data=data)

    # ----------------------------------------------------------

    def apply(self, mov: Move) -> Solution:
        tag, r_from, p_old, d_old, r_to, p_new, d_new, req, pnode, dnode = mov.data

        new_sol = deepcopy(self.sol)
        routeA = new_sol.routes[r_from]
        routeB = new_sol.routes[r_to]

        # Remove delivery first (higher index), then pickup.
        pA = routeA.pop(d_old)
        pB = routeA.pop(p_old)

        # Insert into route B
        routeB.insert(p_new, pnode)
        # delivery index shifts because of pickup insertion
        routeB.insert(d_new, dnode)

        return new_sol

    # ----------------------------------------------------------

    def is_valid(self, mov: Move) -> bool:
        tag, r_from, p_old, d_old, r_to, p_new, d_new, req, pnode, dnode = mov.data

        # We simulate small route changes
        routeA = self.sol.routes[r_from][:]
        routeB = self.sol.routes[r_to][:]

        # Apply the swap to these local copies:

        # Remove in descending order
        del routeA[d_old]
        del routeA[p_old]

        # Insert to route B
        routeB.insert(p_new, pnode)
        routeB.insert(d_new, dnode)

        # Capacity check on BOTH routes
        return bool(
            np.all(calc_route_cargo(self.I, routeA) <= self.I.C) and
            np.all(calc_route_cargo(self.I, routeB) <= self.I.C)
        )

    # ----------------------------------------------------------

    def calc_delta(self, mov: Move) -> float:
        tag, r_from, p_old, d_old, r_to, p_new, d_new, req, pnode, dnode = mov.data
        I = self.I
        dist = I.dist

        # Old routes
        routeA = self.sol.routes[r_from]
        routeB = self.sol.routes[r_to]

        # --------------------------------
        # ARCS REMOVED IN ROUTE A
        # --------------------------------

        # pickup removal
        A_prev = routeA[p_old - 1] if p_old > 0 else 0
        A_next = routeA[p_old + 1] if p_old + 1 < len(routeA) else 0

        # delivery removal: indices shift if d_old > p_old
        d_old_shift = d_old
        # If delivery appears after pickup, after removing pickup first,
        # its index decreases by 1. BUT we compute arc changes BEFORE removal.
        D_prev = routeA[d_old - 1] if d_old > 0 else 0
        D_next = routeA[d_old + 1] if d_old + 1 < len(routeA) else 0

        removed = (
            dist[A_prev, pnode] + dist[pnode, A_next] +
            dist[D_prev, dnode] + dist[dnode, D_next]
        )

        # --------------------------------
        # ARCS ADDED IN ROUTE A (after deletion)
        # Actually: pickup and delivery are removed completely,
        # so route A gains arc (A_prev -> A_next) at pickup position,
        # and (D_prev -> D_next) at delivery.
        # --------------------------------

        added = 0

        # For pickup gap:
        added += dist[A_prev, A_next]

        # For delivery gap:
        added += dist[D_prev, D_next]

        deltaA = added - removed

        # =================================================================
        # NOW route B: inserting pickup at p_new and delivery at d_new
        # =================================================================

        # After pickup insertion:
        B_prev_p = routeB[p_new - 1] if p_new > 0 else 0
        B_next_p = routeB[p_new] if p_new < len(routeB) else 0

        # Delivery insertion (after pickup inserted):
        B_prev_d = routeB[d_new - 1] if d_new > 0 else 0
        B_next_d = routeB[d_new] if d_new < len(routeB) else 0

        removedB = (
            dist[B_prev_p, B_next_p] +
            dist[B_prev_d, B_next_d]
        )

        addedB = (
            dist[B_prev_p, pnode] + dist[pnode, B_next_p] +
            dist[B_prev_d, dnode] + dist[dnode, B_next_d]
        )

        deltaB = addedB - removedB

        return deltaA + deltaB


# =====================================================================
# 2. Two-Opt Neighborhood (PD-compatible)
# =====================================================================

class TwoOptNeighborhood(Neighborhood):

    def generate(self) -> Iterator[Move]:
        for r_idx, route in enumerate(self.sol.routes):
            m = len(route)
            for i in range(m - 2):
                for j in range(i + 2, m):  # non-adjacent to avoid trivial swap
                    yield Move(("2opt", r_idx, i, j))

    # ----------------------------------------------------------

    def apply(self, mov: Move) -> Solution:
        tag, r_idx, i, j = mov.data
        new_sol = deepcopy(self.sol)
        route = new_sol.routes[r_idx]
        route[i:j+1] = reversed(route[i:j+1])
        return new_sol

    # ----------------------------------------------------------

    def is_valid(self, mov: Move) -> bool:
        tag, r_idx, i, j = mov.data
        route = self.sol.routes[r_idx][:]

        # Reverse segment
        route[i:j+1] = reversed(route[i:j+1])

        # Capacity
        if not np.all(calc_route_cargo(self.I, route) <= self.I.C):
            return False

        # Precedence: pickup must occur before delivery
        for idx, node in enumerate(route):
            req = self.I.request_of_node[node]
            if req >= 0:
                # pickup and delivery nodes
                pnode = 2 * req + 1
                dnode = 2 * req + 2
                if dnode in route and pnode in route:
                    if route.index(pnode) > route.index(dnode):
                        return False

        return True

    # ----------------------------------------------------------

    def calc_delta(self, mov: Move) -> float:
        """Delta evaluation for 2-opt: consider boundary arc changes."""
        tag, r_idx, i, j = mov.data
        dist = self.I.dist
        route = self.sol.routes[r_idx]

        # Edges removed: A->x, y->B
        A = route[i - 1] if i > 0 else 0
        x = route[i]
        y = route[j]
        B = route[j + 1] if j + 1 < len(route) else 0

        removed = dist[A, x] + dist[y, B]
        added   = dist[A, y] + dist[x, B]

        # Internal reversal cost changes require full scan
        old_internal = 0
        new_internal = 0

        seg = route[i:j+1]

        for u, v in zip(seg, seg[1:]):
            old_internal += dist[u, v]  # original

        rev = seg[::-1]
        for u, v in zip(rev, rev[1:]):
            new_internal += dist[u, v]  # reversed

        return (added - removed) + (new_internal - old_internal)
