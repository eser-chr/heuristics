#include "solvers.hpp"
#include "utils.hpp"

Solution LS::local_search(
    const Instance &I,
    const Solution &initial_sol,
    const Neighborhood::NeighborhoodFactory &neigh_factories, // list of lambdas kinda
    StepFunction::Func step_function,
    StoppingCriterion &criterion)
{
    Solution sol = initial_sol; // copy
    double f = utils::objective(I, sol);
    int iteration = 0;

    criterion.reset();

    while (!criterion(iteration, f))
    {
        bool improved = false;

        std::vector<std::unique_ptr<Neighborhood>> neighs;
        neighs.reserve(neigh_factories.size());
        for (auto &factory : neigh_factories)
            neighs.push_back(factory(I, sol));

        for (auto &neigh : neighs)
        {
            auto mov = step_function(*neigh);
            if (mov.has_value())
            {
                sol = neigh->apply(*mov);
                f = utils::objective(I, sol);
                improved = true;
                break;
            }
        }

        if (!improved)
            break;

        iteration++;
    }

    return sol;
}
