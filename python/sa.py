
from copy import deepcopy
import random
import math
from typing import List, Type
from structures import Instance, Solution, objective
from neighborhoods import Neighborhood



def simulated_annealing(
    I: Instance,
    sol: Solution,
    neighborhoods_cls: List[Type[Neighborhood]],
    T_start: float = 1.0,
    T_end: float = 1e-3,
    cooling: float = 0.995,
    max_iters: int = 50_000,
) -> Solution:

    sol = deepcopy(sol)
    f = objective(I, sol)

    best_sol = deepcopy(sol)
    best_f = f

    T = T_start

    for it in range(max_iters):

        if T < T_end:
            break

        # Pick one neighborhood at random
        N_cls = random.choice(neighborhoods_cls)
        N = N_cls(I, sol)

        # Randomly sample a move from valid moves
        moves = []
        for mov in N.generate():
            if N.is_valid(mov):
                moves.append(mov)
                if len(moves) >= 20:
                    break

        if not moves:
            T *= cooling
            continue

        mov = random.choice(moves)
        delta = N.calc_delta(mov)

        # Acceptance criterion
        if delta < 0 or random.random() < math.exp(-delta / T):
            sol = N.apply(mov)
            f = f + delta

            if f < best_f:
                best_f = f
                best_sol = deepcopy(sol)

        # Cool down
        T *= cooling

    return best_sol