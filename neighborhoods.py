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
        i, k, l = mov.data
        route = copy(self.sol.routes[i])
        route[k], route[l] = route[l], route[k]
        raise RuntimeError("I am not implemented yet")
    
    # def calc_delta(self, mov: Move) -> float:
    #     r, k, l = mov.data
    #     route = self.sol.routes[r]
    #     I = self.I
    #     dist = I.dist

    #     # nodes before and after positions
    #     x = route[k]
    #     y = route[l]

    #     # A, B around x
    #     A = route[k - 1] if k > 0 else 0
    #     B = route[k + 1] if k + 1 < len(route) else 0

    #     # C, D around y
    #     C = route[l - 1] if l > 0 else 0
    #     D = route[l + 1] if l + 1 < len(route) else 0

    #     # special case: if swapping adjacent nodes k+1 = l
    #     # adjust arcs (A, x, y, D)
    #     if l == k + 1:
    #         # original: A → x → y → D
    #         # new:      A → y → x → D
    #         delta = (
    #             dist[A, y] + dist[y, x] + dist[x, D]
    #             - dist[A, x] - dist[x, y] - dist[y, D]
    #         )
    #         return delta

    #     # general case
    #     delta = (
    #         dist[A, y] + dist[y, B] + dist[C, x] + dist[x, D]
    #         - dist[A, x] - dist[x, B] - dist[C, y] - dist[y, D]
    #     )

    #     return delta
