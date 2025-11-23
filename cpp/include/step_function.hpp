#pragma once
#include <optional>
#include <functional>
#include "neighborhoods.hpp"
#include <random>

namespace StepFunction
{
    using Return_t = std::optional<GenericMove>;
    using Func = std::function<Return_t(const Neighborhood &)>;

    inline Return_t first_improvement(const Neighborhood &N)
    {
        std::vector<GenericMove> moves;
        N.generate(moves);

        for (const auto &m : moves)
        {
            if (!N.is_valid(m))
                continue;
            double d = N.calc_delta(m);
            if (d < 0)
                return m;
        }
        return std::nullopt;
    }

    inline Return_t best_improvement(const Neighborhood &N)
    {
        std::vector<GenericMove> moves;
        N.generate(moves);

        double best_delta = 0;
        Return_t best = std::nullopt;

        for (const auto &m : moves)
        {
            if (!N.is_valid(m))
                continue;
            double d = N.calc_delta(m);
            if (d < best_delta)
            {
                best_delta = d;
                best = m;
            }
        }
        return best;
    }

    inline Return_t random_step(const Neighborhood &N)
    {
        std::vector<GenericMove> moves;
        N.generate(moves);

        std::vector<GenericMove> valid;
        valid.reserve(moves.size());

        for (const auto &m : moves)
            if (N.is_valid(m))
                valid.push_back(m);

        if (valid.empty())
            return std::nullopt;

        static thread_local std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> dist(0, valid.size() - 1);

        return valid[dist(rng)];
    }

} // namespace StepFunction