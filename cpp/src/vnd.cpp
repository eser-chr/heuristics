#include <vector>
#include <memory>
#include <functional>
#include "neighborhoods.hpp"
#include "solvers.hpp"
#include "utils.hpp"

Solution VND::vnd(
    const Instance& I,
    const Solution& initial_sol,
    const Neighborhood::NeighborhoodFactory &neighborhood_factories
) {
    Solution sol = initial_sol;
    double f = utils::objective(I, sol);

    int i = 0;
    int K = (int)neighborhood_factories.size();

    while (i < K) {

        Neighborhood::NeighborhoodFactory single_neigh = { neighborhood_factories[i] };
        MaxIterations stopping(100000);
        Solution new_sol = LS::local_search(
            I,
            sol,
            single_neigh,
            StepFunction::first_improvement,
            stopping
        );

        double f_new = utils::objective(I, new_sol);

        if (f_new < f) {
            sol = new_sol;
            f = f_new;
            i = 0;
        } else {
            i += 1;
        }
    }

    return sol;
}
