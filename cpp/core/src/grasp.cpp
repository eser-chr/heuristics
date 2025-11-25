#include <vector>
#include <algorithm>
#include <random>
#include <functional>
#include "solvers.hpp"
#include "utils.hpp"


// Solution GRASP::randomized_constructor_simple(
//     const Instance &I,
//     double a,
//     double alpha,
//     int max_tries)
// {
//     const int n  = I.n;
//     const int nK = I.nK;

//     // heuristic costs
//     std::vector<double> costs = utils::calc_my_metric(I, a);

//     // argsort(costs)
//     std::vector<int> perm(n);
//     std::iota(perm.begin(), perm.end(), 0);
//     std::sort(perm.begin(), perm.end(),
//               [&](int i, int j) { return costs[i] < costs[j]; });

//     Solution sol;
//     sol.routes.assign(nK, std::vector<int>{});

//     int served = 0;
//     std::vector<bool> used(n, false);

//     static thread_local std::mt19937 rng(std::random_device{}());

//     while (served < I.gamma)
//     {
//         // remaining requests (not yet used)
//         std::vector<int> remaining;
//         remaining.reserve(n);
//         for (int r : perm)
//             if (!used[r])
//                 remaining.push_back(r);

//         if (remaining.empty())
//             break;

//         int k = std::max(1, int(alpha * remaining.size()));
//         if (k > (int)remaining.size())
//             k = (int)remaining.size();

//         std::uniform_int_distribution<int> pick_rcl(0, k - 1);
//         int req = remaining[pick_rcl(rng)];
//         used[req] = true;

//         int pickup = 1 + req;
//         int drop   = 1 + I.n + req;
//         int dem    = I.demands[req];

//         bool inserted = false;

//         for (int t = 0; t < max_tries; ++t)
//         {
//             std::uniform_int_distribution<int> pick_route(0, nK - 1);
//             int vk = pick_route(rng);
//             auto &route = sol.routes[vk];
//             int m = (int)route.size();

//             // handle empty route explicitly
//             if (m == 0)
//             {
//                 if (dem <= I.C) {
//                     route.push_back(pickup);
//                     route.push_back(drop);
//                     inserted = true;
//                     served++;
//                     break;
//                 }
//                 // cannot serve on empty vehicle (capacity too small), try another vehicle/attempt
//                 continue;
//             }

//             // non-empty route: choose ip, jp safely
//             // ip in [0, m-1], jp in [ip+1, m]
//             std::uniform_int_distribution<int> pick_ip(0, m - 1);
//             int ip = pick_ip(rng);

//             std::uniform_int_distribution<int> pick_jp(ip + 1, m);
//             int jp = pick_jp(rng);

//             // build candidate route:
//             std::vector<int> new_r;
//             new_r.reserve(m + 2);

//             // [0, ip)
//             new_r.insert(new_r.end(), route.begin(), route.begin() + ip);
//             // pickup
//             new_r.push_back(pickup);
//             // [ip, jp)
//             new_r.insert(new_r.end(), route.begin() + ip, route.begin() + jp);
//             // drop
//             new_r.push_back(drop);
//             // [jp, m)
//             new_r.insert(new_r.end(), route.begin() + jp, route.end());

//             auto cargo = utils::calc_route_cargo(I, new_r);
//             bool ok = true;
//             for (int c : cargo)
//             {
//                 if (c < 0 || c > I.C)
//                 {
//                     ok = false;
//                     break;
//                 }
//             }

//             if (ok)
//             {
//                 route = std::move(new_r);
//                 inserted = true;
//                 served++;
//                 break;
//             }
//         }

//         // failed to insert this request in max_tries attempts → skip it
//         if (!inserted)
//             continue;
//     }

//     return sol;
// }


Solution GRASP::randomized_constructor_simple(
    const Instance &I,
    double a,
    double alpha)
{
    const int n  = I.n;
    const int nK = I.nK;

    std::vector<double> costs = utils::calc_my_metric(I, a);

    std::vector<int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);
    std::sort(perm.begin(), perm.end(),
              [&](int i, int j){ return costs[i] < costs[j]; });

    Solution sol;
    sol.routes.assign(nK, {});

    std::vector<bool> used(n, false);
    int served = 0;

    static thread_local std::mt19937 rng(std::random_device{}());

    while (served < I.gamma)
    {
        // build remaining candidate list
        std::vector<int> cand;
        for (int r : perm)
            if (!used[r])
                cand.push_back(r);
        if (cand.empty()) break;

        // determine RCL by threshold
        double c_min = costs[cand[0]];
        double c_max = costs[cand.back()];
        double thresh = c_min + alpha * (c_max - c_min);

        std::vector<int> RCL;
        for (int r : cand)
            if (costs[r] <= thresh)
                RCL.push_back(r);

        if (RCL.empty())  // shouldn't happen, but safe
            RCL.push_back(cand[0]);

        // pick request from RCL at random
        std::uniform_int_distribution<int> pick_idx(0, RCL.size()-1);
        int req = RCL[pick_idx(rng)];
        used[req] = true;

        int pickup = 1 + req;
        int drop   = 1 + I.n + req;
        int dem    = I.demands[req];

        // find best feasible insertion for (pickup, drop)
        double best_inc = 1e18;
        int best_r = -1, best_ip = -1, best_jp = -1;

        for (int r = 0; r < nK; r++)
        {
            auto &route = sol.routes[r];
            int m = route.size();

            // attempt all insertions
            for (int ip = 0; ip <= m; ip++)
            {
                for (int jp = ip+1; jp <= m+1; jp++)
                {
                    std::vector<int> new_r;
                    new_r.reserve(m+2);

                    new_r.insert(new_r.end(), route.begin(), route.begin()+ip);
                    new_r.push_back(pickup);
                    new_r.insert(new_r.end(), route.begin()+ip, route.begin()+jp);
                    new_r.push_back(drop);
                    new_r.insert(new_r.end(), route.begin()+jp, route.end());

                    auto cargo = utils::calc_route_cargo(I, new_r);
                    bool ok=true;
                    for (int c : cargo)
                        if (c < 0 || c > I.C) { ok=false; break; }
                    if (!ok) continue;

                    // compute incremental cost
                    double inc = utils::route_distance(I, new_r)
                               - utils::route_distance(I, route);

                    if (inc < best_inc)
                    {
                        best_inc = inc;
                        best_r = r;
                        best_ip = ip;
                        best_jp = jp;
                    }
                }
            }
        }

        if (best_r == -1)
            continue; // cannot insert, skip

        // apply best insertion
        auto &route = sol.routes[best_r];
        std::vector<int> new_r;
        int m = route.size();
        new_r.reserve(m+2);
        new_r.insert(new_r.end(), route.begin(), route.begin()+best_ip);
        new_r.push_back(pickup);
        new_r.insert(new_r.end(), route.begin()+best_ip, route.begin()+best_jp);
        new_r.push_back(drop);
        new_r.insert(new_r.end(), route.begin()+best_jp, route.end());
        route = std::move(new_r);

        served++;
    }

    return sol;
}



Solution GRASP::grasp(
    const Instance &I,
    std::function<Solution(const Instance &)> randomized_constructor,
    const Neighborhood::NeighborhoodFactories &neighborhoods,
    StepFunction::Func step_function,
    StoppingCriterion &stopping_outer,
    StoppingCriterion &stopping_local)
{
    Solution best_sol;                         // final result
    double best_f = std::numeric_limits<double>::infinity();

    int step = 0;

    // reset local-search stop criterion every restart
    stopping_local.reset();
    stopping_outer.reset();
    
    while (!stopping_outer(step, best_f))
    {
        // === construct a new initial solution ===
        Solution sol0 = randomized_constructor(I);

        stopping_local.reset();
        Solution sol1 = LS::local_search(
            I,
            sol0,
            neighborhoods[step%neighborhoods.size()], //rotate neighborhood
            step_function,
            stopping_local);

        double f1 = utils::objective(I, sol1);

        if (f1 < best_f)
        {
            best_f = f1;
            best_sol = sol1;
        }

        step++;
    }

    return best_sol;
}
