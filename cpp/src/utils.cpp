#include "utils.hpp"
#include <algorithm>
namespace utils
{

    std::vector<int> calc_route_cargo(
        const Instance &inst,
        const std::vector<int> &route)
    {
        std::vector<int> out(route.size());
        int load = 0;

        for (size_t t = 0; t < route.size(); t++)
        {
            int node = route[t];
            load += inst.load_change[node];
            out[t] = load;
        }
        return out;
    }

    int route_distance(const Instance &inst, const std::vector<int> &route)
    {
        if (route.empty())
            return 0;

        int d = inst.dist[0][route[0]];
        for (size_t i = 0; i + 1 < route.size(); i++)
            d += inst.dist[route[i]][route[i + 1]];
        d += inst.dist[route.back()][0];

        return d;
    }

    std::vector<double> all_route_distances(
        const Instance &inst,
        const Solution &sol)
    {
        std::vector<double> res(sol.routes.size());
        for (size_t i = 0; i < sol.routes.size(); i++)
            res[i] = (double)route_distance(inst, sol.routes[i]);
        return res;
    }

    double jain_fairness(const std::vector<double> &dists)
    {
        if (dists.empty())
            throw std::runtime_error("dist has length 0!");

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
            ss << "Division by zero in Jain fairness. "
               << "len=" << dists.size() << ", sq_sum=" << sq_sum;
            throw std::runtime_error(ss.str());
        }

        return num / den;
    }

    bool check_route_feasible(const Instance &inst,
                              const std::vector<int> &route)
    {
        int load = 0;
        std::unordered_set<int> picked;
        std::unordered_set<int> dropped;

        for (int node : route)
        {

            int req = inst.request_of_node[node];
            if (req < 0)
                return false; // invalid node inside route

            load += inst.load_change[node];
            if (load > inst.C || load < 0)
                return false; // capacity violation

            if (inst.load_change[node] > 0)
                picked.insert(req);
            else
            {
                if (!picked.count(req))
                    return false; // drop before pickup
                dropped.insert(req);
            }
        }

        // must have at least gamma drops
        return (int)dropped.size() >= inst.gamma;
    }

    double objective(const Instance &inst, const Solution &sol)
    {
        auto dists = all_route_distances(inst, sol);

        double sum_dist = std::accumulate(dists.begin(), dists.end(), 0.0);
        double fairness = jain_fairness(dists);

        return sum_dist + inst.rho * (1.0 - fairness);
    }

    std::vector<double> calc_my_metric(const Instance &I, double a)
    {
        int n = I.n;

        // Solo trip from depot pickup delivery depot.
        std::vector<double> solo(n);
        for (int req = 0; req < n; ++req)
        {
            int p = 1 + req;
            int d = 1 + n + req;
            int s = I.dist[0][p] + I.dist[p][d] + I.dist[d][0];
            solo[req] = static_cast<double>(s);
        }

        double max_dist = 0.0;
        for (double v : solo)
            max_dist = std::max(max_dist, v);

        int max_dem_int = 0;
        for (int c : I.demands)
            max_dem_int = std::max(max_dem_int, c);

        if (max_dist == 0.0)
            max_dist = 1.0;
        if (max_dem_int == 0)
            max_dem_int = 1;

        double max_dem = static_cast<double>(max_dem_int);

        std::vector<double> costs(n);
        for (int i = 0; i < n; ++i)
        {
            double dist_norm = solo[i] / max_dist;
            double dem_norm = static_cast<double>(I.demands[i]) / max_dem;
            costs[i] = a * dist_norm + (1.0 - a) * dem_norm;
        }
        return costs;
    }


} // namespace utils

namespace numerical
{
    template <typename T>
    std::vector<int> argsort(const std::vector<T> &org)
    {
        std::vector<int> perm(org.size());
        std::iota(perm.begin(), perm.end(), 0);
        std::sort(perm.begin(), perm.end(),
                  [&](int i, int j)
                  { return org[i] < org[j]; });
        return perm;
    }
    
    
};
template std::vector<int> numerical::argsort<double>(const std::vector<double>&);
template std::vector<int> numerical::argsort<int>(const std::vector<int>&);