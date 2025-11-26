#pragma once

#include <vector>
#include <random>
#include <numeric>
#include <unordered_set>
#include <stdexcept>
#include <sstream>

#include "instance.hpp"
#include "solution.hpp"

namespace utils
{

    std::vector<int> calc_route_cargo(
        const Instance &inst, const std::vector<int> &route);

    int route_distance(
        const Instance &inst, const std::vector<int> &route);

    std::vector<double> all_route_distances(
        const Instance &inst, const Solution &sol);

    double jain_fairness(Instance const & I, 
        const std::vector<double> &dists);

    bool check_route_feasible(
        const Instance &inst, const std::vector<int> &route);

    double objective(
        const Instance &inst, const Solution &sol);

    std::vector<double> calc_my_metric(const Instance &I, double a);
} // namespace utils

namespace numerical
{
    template <typename T>
    std::vector<int> argsort(const std::vector<T> &org);

    template<typename T>
    T select_uniformly(const std::vector<T> & org, std::mt19937 & rng);

    // double calc_dist2(const gt::Coords &p1, const gt::Coords &p2);
};

namespace gt
{
    template <typename T>
    using Matrix = std::vector<std::vector<T>>;

    using Coords = std::pair < double,double>;
};