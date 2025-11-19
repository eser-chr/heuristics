from __future__ import annotations
from neighborhoods import *
from typing import Callable, Optional, Any
from abc import abstractmethod


# ------------------------------------------------------------
# Base Functor Interface
# ------------------------------------------------------------


class StoppingCriterion(ABC):
    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def __call__(self, iteration: int, f: float) -> bool:
        """
        Returns True if Local Search should stop.
        """
        ...

class MaxIterations(StoppingCriterion):
    def __init__(self, max_iters: int):
        self.max_iters = max_iters

    def reset(self) -> None:
        pass

    def __call__(self, iteration: int, f: float) -> bool:
        return iteration >= self.max_iters


class ObjectiveThreshold(StoppingCriterion):
    def __init__(self, threshold: float):
        self.threshold = threshold

    def reset(self) -> None:
        pass

    def __call__(self, iteration: int, f: float) -> bool:
        return f <= self.threshold


class ImprovementThreshold(StoppingCriterion):
    def __init__(self, eps: float):
        self.eps = eps
        self.last_f: Optional[float] = None

    def reset(self) -> None:
        self.last_f = None

    def __call__(self, iteration: int, f: float) -> bool:
        if self.last_f is None:
            self.last_f = f
            return False
        diff = abs(self.last_f - f)
        self.last_f = f
        return diff < self.eps


# ------------------------------------------------------------
# Combinators (Functor Composition)
# ------------------------------------------------------------


class AnyCriterion(StoppingCriterion):
    """
    Stops if ANY of its child criteria return True.
    """

    def __init__(self, *criteria: StoppingCriterion):
        self.criteria = criteria

    def reset(self) -> None:
        for c in self.criteria:
            c.reset()

    def __call__(self, iteration: int, f: float) -> bool:
        return any(c(iteration, f) for c in self.criteria)


class AllCriterion(StoppingCriterion):
    """
    Stops only if ALL of its child criteria return True.
    """

    def __init__(self, *criteria: StoppingCriterion):
        self.criteria = criteria

    def reset(self) -> None:
        for c in self.criteria:
            c.reset()

    def __call__(self, iteration: int, f: float) -> bool:
        return all(c(iteration, f) for c in self.criteria)


class StepFunction:
    Return_t = Optional[Move]
    Func_t = Callable[[Neighborhood], Return_t]

    @staticmethod
    def first_improvement(neighborhood: Neighborhood) -> Return_t:
        return None

    @staticmethod
    def best_improvement(neighborhood: Neighborhood) -> Return_t:
        return None

    @staticmethod
    def random_step(neighborhood: Neighborhood) -> Return_t:
        return None


def local_search(
    I: Instance,
    sol: Solution,
    neighborhoods_cls: List[type[Neighborhood]],
    step_function: StepFunction.Func_t,
    stopping_criterion: StoppingCriterion,
):

    sol = deepcopy(sol)
    f = objective(I, sol)
    iteration: int = 0

    while not stopping_criterion(iteration, f):

        improved = False
        # rebuild neighborhoods for the updated solution
        n_objs = [n_cls(I, sol) for n_cls in neighborhoods_cls]

        for neigh in n_objs:
            mov = step_function(neigh)
            if mov is not None:
                sol = neigh.apply(mov)
                improved = True
                break

        # I go over all neighborhoods but no improvement is obsereved.
        if not improved:
            break
        iteration += 1

    return sol
