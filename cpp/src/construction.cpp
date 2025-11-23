#include <vector>
#include <algorithm>

#include "utils.hpp"
#include "solvers.hpp"


Solution DRC::construction(
    const Instance& I,
    double a,
    double sigma_factor,
    bool is_random
) {
    int n  = I.n;
    int nK = I.nK;

    std::vector<double> costs = utils::calc_my_metric(I, a);
    std::vector<double> noisy = costs;

    static thread_local std::mt19937 rng(std::random_device{}());

    if (is_random) {
        // compute stddev of costs
        double mean = std::accumulate(costs.begin(), costs.end(), 0.0) / costs.size();
        double var  = 0.0;
        for (double x : costs) {
            double d = x - mean;
            var += d * d;
        }
        var /= costs.size();
        double stddev = std::sqrt(var);

        double base = (stddev > 0.0 ? stddev : 1.0);
        double sigma = sigma_factor * std::max(base, 1e-6);

        std::normal_distribution<double> dist(0.0, sigma);
        for (int i = 0; i < n; ++i) {
            noisy[i] += dist(rng);
        }
    }

    // argsort(noisy)
    std::vector<int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);
    std::sort(perm.begin(), perm.end(),
              [&](int i, int j) { return noisy[i] < noisy[j]; });

    int gamma = std::min(I.gamma, n);
    std::vector<int> important(perm.begin(), perm.begin() + gamma);

    // per_track_requests
    std::vector<std::vector<int>> per_track_requests(nK);
    for (int t = 0; t < nK; ++t) {
        for (int idx = t; idx < (int)important.size(); idx += nK) {
            per_track_requests[t].push_back(important[idx]);
        }
    }

    std::vector<std::vector<int>> routes;
    routes.reserve(nK);

    for (int track = 0; track < nK; ++track) {
        std::vector<int> route;
        int cargo = 0;
        std::vector<int> active;

        for (int req : per_track_requests[track]) {
            int pickup = 1 + req;
            int dem = I.demands[req];

            // capacity check: drop "heaviest" (Python actually used min-demand; keep semantics)
            if (cargo + dem > I.C) {
                if (active.empty()) {
                    continue;
                }
                // Python: heaviest = min(active, key=lambda r: demands[r])
                int heaviest = active[0];
                for (int r : active) {
                    if (I.demands[r] < I.demands[heaviest]) {
                        heaviest = r;
                    }
                }
                // remove from active
                active.erase(std::remove(active.begin(), active.end(), heaviest),
                             active.end());
                route.push_back(1 + I.n + heaviest);
                cargo -= I.demands[heaviest];
            }

            route.push_back(pickup);
            active.push_back(req);
            cargo += dem;
        }

        // drop remaining active in descending demand order
        std::sort(active.begin(), active.end(),
                  [&](int r1, int r2) {
                      return I.demands[r1] > I.demands[r2];
                  });
        for (int r : active) {
            route.push_back(1 + I.n + r);
        }

        routes.push_back(std::move(route));
    }

    // Ensure we serve at least gamma requests
    std::unordered_set<int> served;
    for (const auto& r : routes) {
        for (int node : r) {
            int req = I.request_of_node[node];
            if (req >= 0) served.insert(req);
        }
    }

    if ((int)served.size() < gamma) {
        std::vector<int> missing;
        missing.reserve(n);
        for (int req : perm) {
            if (!served.count(req)) missing.push_back(req);
        }

        for (int req : missing) {
            if ((int)served.size() >= gamma) break;
            int pickup = 1 + req;
            int drop   = 1 + I.n + req;
            for (int k = 0; k < nK; ++k) {
                auto new_r = routes[k];
                new_r.push_back(pickup);
                new_r.push_back(drop);
                auto cargo_vec = utils::calc_route_cargo(I, new_r);

                bool ok = true;
                for (int c : cargo_vec) {
                    if (c < 0 || c > I.C) { ok = false; break; }
                }
                if (ok) {
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
