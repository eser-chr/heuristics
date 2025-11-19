from neighborhoods import *
from local_search import (
    local_search,
    StepFunction,
    StoppingCriterion,
    ImprovementThreshold,
    MaxIterations,
)


def vnd(I: Instance, sol: Solution, neighborhoods_cls: List[type[Neighborhood]]):
    sol = deepcopy(sol)
    f = objective(I, sol)

    i = 0
    K = len(neighborhoods_cls)

    while i < K:
        new_sol = local_search(
            I,
            sol,
            [neighborhoods_cls[i]],
            StepFunction.first_improvement,
            MaxIterations(10**5),
        )

        f_new = objective(I, new_sol)

        if f_new < f:
            sol = new_sol
            f = f_new
            i = 0
        else:
            i += 1

    return sol
