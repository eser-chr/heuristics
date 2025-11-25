#include "neighborhoods.hpp"
#include "solvers.hpp"
#include "utils.hpp"

Solution SA::simulated_annealing(
    const Instance &I,
    const Solution &initial_sol,
    const Neighborhood::NeighborhoodFactories &neighborhood_factories,
    double T_start,
    double T_end,
    double cooling,
    int max_iters)
{
    if (neighborhood_factories.empty())
    {
        return initial_sol;
    }

    // RNG
    static thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    Solution sol = initial_sol;
    double f = utils::objective(I, sol);

    Solution best_sol = sol;
    double best_f = f;

    double T = T_start;

    for (int it = 0; it < max_iters; ++it)
    {
        if (T < T_end)
            break;

        // pick one neighborhood at random
        std::uniform_int_distribution<int> neigh_dist(0, (int)neighborhood_factories.size() - 1);
        int idx_neigh = neigh_dist(rng);

        auto N = neighborhood_factories[idx_neigh](I, sol);

        // generate up to 20 valid moves
        std::vector<GenericMove> all_moves;
        N->generate(all_moves);

        std::vector<GenericMove> moves;
        moves.reserve(20);
        for (const auto &mv : all_moves)
        {
            if (N->is_valid(mv))
            {
                moves.push_back(mv);
                if ((int)moves.size() >= 20)
                    break;
            }
        }

        if (moves.empty())
        {
            T *= cooling;
            continue;
        }

        // choose one random move among the sampled valid ones
        std::uniform_int_distribution<int> move_dist(0, (int)moves.size() - 1);
        int midx = move_dist(rng);
        const auto &mov = moves[midx];

        double delta = N->calc_delta(mov);

        bool accept = false;
        if (delta < 0.0)
        {
            accept = true;
        }
        else
        {
            double p = std::exp(-delta / T);
            if (uni(rng) < p)
                accept = true;
        }

        if (accept)
        {
            sol = N->apply(mov);
            f += delta; // mirrors Python: f = f + delta

            if (f < best_f)
            {
                best_f = f;
                best_sol = sol;
            }
        }

        T *= cooling;
    }

    return best_sol;
}
