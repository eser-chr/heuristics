from neighborhoods import *
from local_search import StoppingCriterion, MaxIterations, StepFunction, local_search
from typing import Callable, Optional
from construction import calc_my_metric
import random
        

# def randomized_constructor(
#     I: Instance,
#     alpha: float = 0.3
# ) -> Solution:

#     sol = Solution(routes=[[] for _ in range(I.nK)])

#     # --- score each request by direct pickup->drop distance ---
#     scores = np.zeros(I.n)
#     for r in range(I.n):
#         pu = 1 + r
#         dr = 1 + I.n + r
#         scores[r] = I.dist[pu, dr]  # simple greedy metric

#     # --- sort requests by greedy score ---
#     req_order = np.argsort(scores)

#     served = 0
#     used = set()

#     for _ in range(I.n):

#         # remaining requests
#         remaining = [r for r in req_order if r not in used]
#         if not remaining:
#             break

#         # greedy interval
#         k = max(1, int(alpha * len(remaining)))
#         rcl = remaining[:k]          # restricted candidate list
#         req = random.choice(rcl)     # pick random request from RCL
#         used.add(req)

#         pickup = 1 + req
#         drop = 1 + I.n + req
#         demand = I.demands[req]

#         best_route = None
#         best_pos = None
#         best_delta = math.inf

#         # --- try all vehicles and insertion positions ---
#         for vk in range(I.nK):
#             route = sol.routes[vk]

#             m = len(route)
#             for ip in range(m + 1):
#                 for jp in range(ip + 1, m + 2):

#                     # candidate route
#                     new_route = route[:ip] + [pickup] + route[ip:jp] + [drop] + route[jp:]

#                     cargo = calc_route_cargo(I, new_route)
#                     if np.any(cargo > I.C) or np.any(cargo < 0):
#                         continue  # infeasible

#                     d_old = route_distance(I, route)
#                     d_new = route_distance(I, new_route)
#                     delta = d_new - d_old

#                     if delta < best_delta:
#                         best_delta = delta
#                         best_route = vk
#                         best_pos = (ip, jp)

#         if best_route is None:
#             # cannot insert further; stop early
#             break

#         # --- apply best insertion ---
#         ip, jp = best_pos
#         r = sol.routes[best_route]
#         sol.routes[best_route] = r[:ip] + [pickup] + r[ip:jp] + [drop] + r[jp:]
#         served += 1

#         if served >= I.gamma:
#             break

#     return sol

def randomized_constructor_simple(
    I: Instance,
    a: float = 0.5,
    alpha: float = 0.3,
    max_tries: int = 20
) -> Solution:

    costs = calc_my_metric(I, a)

    # sort requests by heuristic cost
    perm = np.argsort(costs)
    n = I.n
    
    sol = Solution(routes=[[] for _ in range(I.nK)])
    served = 0
    used = set()

    while served < I.gamma:

        # --- RCL creation ---
        remaining = [r for r in perm if r not in used]
        if not remaining:
            break

        k = max(1, int(alpha * len(remaining)))
        rcl = remaining[:k]

        req = random.choice(rcl)
        used.add(req)

        pickup = 1 + req
        drop = 1 + I.n + req

        inserted = False

        # --- Try random routes and random insertion positions ---
        for _ in range(max_tries):

            vk = random.randrange(I.nK)
            route = sol.routes[vk]
            m = len(route)

            # generate random insertion points
            ip = random.randint(0, m)
            jp = random.randint(ip + 1, m + 1)

            new_r = route[:ip] + [pickup] + route[ip:jp] + [drop] + route[jp:]
            cargo = calc_route_cargo(I, new_r)

            if np.all((0 <= cargo) & (cargo <= I.C)):
                sol.routes[vk] = new_r
                inserted = True
                served += 1
                break

        # if failed to insert this request, skip it and continue
        if not inserted:
            continue

    return sol

        
def grasp(
    I: Instance,
    randomized_constructor: Callable[[Instance], Solution],
    neighborhoods_cls: List[type[Neighborhood]],
    step_function: StepFunction.Func_t,
    stopping_criterion_outer: StoppingCriterion,
    stopping_criterion_local_search: StoppingCriterion,    
) -> Optional[Solution]:

    best_sol = None # maybe add a greedy construction to be safe.
    best_f = math.inf

    step = 0
    while not stopping_criterion_outer(step, best_f):
        sol0 = randomized_constructor(I)        # random constructive phase
        sol1 = local_search(                   # local search refinement
            I,
            sol0,
            neighborhoods_cls,
            step_function,
            stopping_criterion_local_search,
        )

        f1 = objective(I, sol1)

        if f1 < best_f:
            best_f = f1
            best_sol = sol1
            
        step+=1

    return best_sol
