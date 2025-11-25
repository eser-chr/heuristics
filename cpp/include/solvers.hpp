#pragma once
#include <vector>
#include "instance.hpp"
#include "solution.hpp"
#include "neighborhoods.hpp"
#include "step_function.hpp"
#include "stopping_criteria.hpp"

namespace DRC // Deterministic Random Construction
{

    Solution construction(
        const Instance &I,
        double a,
        double sigma_factor = 0.1,
        bool is_random = false);
};
namespace BS
{
    struct BeamState
    {
        double score;
        std::vector<int> route;
        int cargo;
        std::vector<int> active;
        std::vector<int> remaining;
    };

    Solution beam_search(const Instance &I, double a, int beam_width = 5);

};
namespace LS
{
    Solution local_search(
        const Instance &I,
        const Solution &initial_sol,
        const Neighborhood::NeighborhoodFactory &neigh_factories,
        StepFunction::Func step_function,
        StoppingCriterion &criterion);
};

namespace VND
{

    Solution vnd(
        const Instance &I,
        const Solution &initial_sol,
        const Neighborhood::NeighborhoodFactories &neighborhood_factories,
        StepFunction::Func step_function,
        StoppingCriterion& stopping_criterion
    );

};
namespace GRASP
{

    Solution randomized_constructor_simple(
        const Instance &I,
        double a = 0.5,
        double alpha = 0.3,
        int max_tries = 20);

    Solution grasp(
        const Instance &I,
        std::function<Solution(const Instance &)> randomized_constructor,
        const Neighborhood::NeighborhoodFactories &neighborhoods,
        StepFunction::Func step_function,
        StoppingCriterion &stopping_outer,
        StoppingCriterion &stopping_local);
};

namespace SA
{
    Solution simulated_annealing(
        const Instance &I,
        const Solution &initial_sol,
        const Neighborhood::NeighborhoodFactories &neighborhood_factories,
        double T_start = 1.0,
        double T_end = 1e-3,
        double cooling = 0.995,
        int max_iters = 50'000);
};
