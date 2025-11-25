#include <vector>
#include <algorithm>

#include "utils.hpp"
#include "solvers.hpp"

void add_random_noise(const Instance &I, std::vector<double> noisy, const std::vector<double> &costs, double sigma_factor)
{
    static thread_local std::mt19937 rng(std::random_device{}());
    double mean = std::accumulate(costs.begin(), costs.end(), 0.0) / costs.size();
    double var = 0.0;
    for (double x : costs)
    {
        double d = x - mean;
        var += d * d;
    }
    var /= costs.size();
    double stddev = std::sqrt(var);

    double base = (stddev > 0.0 ? stddev : 1.0);
    double sigma = sigma_factor * std::max(base, 1e-6);

    std::normal_distribution<double> dist(0.0, sigma);
    for (int i = 0; i < I.n; ++i)
    {
        noisy[i] += dist(rng);
    }
}

gt::Matrix<int> get_ptrack_requests(int nK, const std::vector<int> requests)
{
    std::vector<std::vector<int>> per_track_requests(nK);
    for (int t = 0; t < nK; ++t)
    {
        for (int idx = t; idx < (int)requests.size(); idx += nK)
        {
            per_track_requests[t].push_back(requests[idx]);
        }
    }
    return per_track_requests;
}

// bool are_all_requests_served(const Instance & I, const gt::Matrix<int>& routes)
// {
//     std::unordered_set<int> served;
//     for (const auto &r : routes)
//     {
//         for (int node : r)
//         {
//             int req = I.request_of_node[node];
//             if (req >= 0)
//                 served.insert(req);
//         }
//     }
// }

Solution DRC::construction(
    const Instance &I,
    double a,
    double sigma_factor,
    bool is_random)
{

    std::vector<double> costs = utils::calc_my_metric(I, a);
    std::vector<double> noisy = costs;

    if (is_random)
        add_random_noise(I, noisy, costs, sigma_factor);

    auto perm = numerical::argsort(noisy);
    std::vector<int> important(perm.begin(), perm.begin() + I.gamma); // Select first gamma requests to fullfill.
    auto per_track_requests = get_ptrack_requests(I.nK, important);

    gt::Matrix<int> routes;
    routes.reserve(I.nK);

    for (int track = 0; track < I.nK; ++track)
    {
        std::vector<int> route;
        std::vector<int> active;
        int cargo = 0;

        auto less_dem = [&](int r1, int r2)
        { return I.demands[r1] < I.demands[r2]; };
        auto greater_dem = [&](int r1, int r2)
        { return I.demands[r1] > I.demands[r2]; };

        for (int req : per_track_requests[track])
        {
            int pickup = 1 + req;
            int dem = I.demands[req];

            if (cargo + dem > I.C)
            {
                // find the lightest request and deliver it i.e push deliver
                // node to the route.
                int lightest =
                    *std::min_element(active.begin(), active.end(), less_dem);

                active.erase(std::remove(active.begin(), active.end(), lightest),
                             active.end());
                route.push_back(1 + I.n + lightest);
                cargo -= I.demands[lightest];
            }

            route.push_back(pickup);
            active.push_back(req);
            cargo += dem;
        }

        // drop remaining active in descending demand order
        // std::sort(active.begin(), active.end(), greater_dem);

        int last = route.empty() ? 0 : route.back(); // current end node

        while (!active.empty())
        {
            int best_r = -1;
            double best_d = 1e18;

            for (int r : active)
            {
                int deliver_node = 1 + I.n + r;
                double d = I.dist[last][deliver_node];
                if (d < best_d)
                {
                    best_d = d;
                    best_r = r;
                }
            }

            // append best delivery
            int deliver_node = 1 + I.n + best_r;
            route.push_back(deliver_node);

            // update last
            last = deliver_node;

            // remove it from active
            active.erase(std::remove(active.begin(), active.end(), best_r),
                         active.end());
        }

        for (int r : active)
            route.push_back(1 + I.n + r);

        routes.push_back(std::move(route));
    }

    // Ensure we serve at least gamma requests
    std::unordered_set<int> served;
    for (const auto &r : routes)
    {
        for (int node : r)
        {
            int req = I.request_of_node[node];
            if (req >= 0)
                served.insert(req);
        }
    }

    if ((int)served.size() < I.gamma)
    {
        std::vector<int> missing;
        missing.reserve(I.n);
        for (int req : perm)
        {
            if (!served.count(req))
                missing.push_back(req);
        }

        for (int req : missing)
        {
            if ((int)served.size() >= I.gamma)
                break;
            int pickup = 1 + req;
            int drop = 1 + I.n + req;
            for (int k = 0; k < I.nK; ++k)
            {
                auto new_r = routes[k];
                new_r.push_back(pickup);
                new_r.push_back(drop);
                auto cargo_vec = utils::calc_route_cargo(I, new_r);

                bool ok = true;
                for (int c : cargo_vec)
                {
                    if (c < 0 || c > I.C)
                    {
                        ok = false;
                        break;
                    }
                }
                if (ok)
                {
                    routes[k] = std::move(new_r);
                    served.insert(req);
                    break;
                }
            }
        }
    }

    Solution sol;
    sol.routes = std::move(routes);
    return sol;
}
