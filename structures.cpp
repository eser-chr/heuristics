#pragma once
#include "instance.hpp"
#include "solution.hpp"
#include <unordered_set>
#include <sstream>
#include <iostream>
#include <numeric>


std::vector<int> calc_route_cargo(const Instance &I, const std::vector<int> &route)
{
    int cargo = 0;
    std::vector<int> out;
    out.reserve(route.size());

    for (int node : route)
    {
        cargo += I.load_change[node]; // +d_i at pickup, -d_i at delivery
        out.push_back(cargo);
    }
    return out;
}

int route_distance(const Instance &inst, const std::vector<int> &route)
{
    if (route.empty())
        return 0;

    int d = inst.dist[0][route[0]]; // depot -> first

    for (size_t i = 0; i + 1 < route.size(); ++i)
        d += inst.dist[route[i]][route[i + 1]];

    d += inst.dist[route.back()][0]; // last -> depot
    return d;
}

std::vector<double> all_route_distances(const Instance &inst, const Solution &sol)
{
    std::vector<double> dists;
    dists.reserve(sol.routes.size());

    for (const auto &r : sol.routes)
        dists.push_back(static_cast<double>(route_distance(inst, r)));

    return dists;
}

double jain_fairness(const std::vector<double> &dists)
{
    if (dists.empty())
        throw std::runtime_error("dist has length 0!!");

    double sum = 0.0;
    double sq_sum = 0.0;

    for (double x : dists)
    {
        sum += x;
        sq_sum += x * x;
    }

    double num = sum * sum;
    double den = dists.size() * sq_sum;

    if (den == 0.0)
    {
        std::stringstream ss;
        ss << "Division with zero during calc of jain fairness. "
           << "len(dists)=" << dists.size()
           << ", sq_sum=" << sq_sum;
        throw std::runtime_error(ss.str());
    }

    return num / den;
}


bool check_route_feasible(const Instance& inst, const std::vector<int>& route) {
    int load = 0;
    std::unordered_set<int> picked;
    std::unordered_set<int> dropped;

    for (int node : route) {
        int req = inst.request_of_node[node];
        if (req < 0)
            return false;   // illegal node (depot or invalid location)

        load += inst.load_change[node];
        if (load > inst.C || load < 0)
            return false;   // capacity violation

        if (inst.load_change[node] > 0) {
            // pickup
            picked.insert(req);
        } else {
            // drop
            if (!picked.count(req))
                return false;   // drop before pickup
            dropped.insert(req);
        }
    }

    // must serve at least gamma requests
    if ((int)dropped.size() < inst.gamma)
        return false;

    return true;
}

double objective(const Instance& inst, const Solution& sol) {
    auto dists = all_route_distances(inst, sol);
    return std::accumulate(dists.begin(), dists.end(), 0.0)
         + inst.rho * (1.0 - jain_fairness(dists));
}
