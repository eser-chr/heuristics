# from __future__ import annotations
from neighborhoods import *
from local_search import StepFunction
from typing import List, Optional, Any
from copy import deepcopy
import math


def default_tabu_attribute(mov: Move) -> Any:
    return mov.data


def tabu_search(
    I: Instance,
    sol: Solution,
    neighborhoods_cls: List[type[Neighborhood]],
    step_function: StepFunction.Func_t,
    tabu_tenure: int = 20,
    max_iters: int = 500,
    aspiration_factor: float = 0.99,
    tabu_attribute_func = default_tabu_attribute,
) -> Solution:

    sol = deepcopy(sol)
    f = objective(I, sol)
    best_sol = deepcopy(sol)
    best_f = f
    tabu = {}

    for it in range(max_iters):
        neigh_objs = [cls(I, sol) for cls in neighborhoods_cls]

        best_mov = None
        best_delta = math.inf
        best_neigh = None

        for neigh in neigh_objs:
            for mov in neigh.generate():
                if not neigh.is_valid(mov):
                    continue

                delta = neigh.calc_delta(mov)
                tabu_key = tabu_attribute_func(mov)
                tabu_active = tabu_key in tabu and tabu[tabu_key] > it
                aspirational = (f + delta) < best_f * aspiration_factor

                if tabu_active and not aspirational:
                    continue

                if delta < best_delta:
                    best_delta = delta
                    best_mov = mov
                    best_neigh = neigh

        if best_mov is None:
            break
        
        if best_neigh is None:
            raise RuntimeError("Something went wrong. best neighbor is None")
        
        sol = best_neigh.apply(best_mov)
        f = f + best_delta

        if f < best_f:
            best_f = f
            best_sol = deepcopy(sol)

        tabu_key = tabu_attribute_func(best_mov)
        tabu[tabu_key] = it + tabu_tenure

        expired = [k for k, exp in tabu.items() if exp <= it]
        for k in expired:
            del tabu[k]

    return best_sol
