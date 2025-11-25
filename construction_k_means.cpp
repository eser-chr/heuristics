#include <vector>
#include <algorithm>
#include "instance.hpp"
#include "solution.hpp"
#include "solvers.hpp"



std::vector<int> kmeans_assign(
    const Instance& I, int K, int iters = 20)
{
    int n = I.n;
    std::vector<int> assign(n, 0);

    // initialize centers by picking K random requests
    std::vector<double> cx(K), cy(K);
    for (int k = 0; k < K; k++) {
        int r = rand() % n;
        cx[k] = I.coords[r].first;
        cy[k] = I.coords[r].second;
    }

    for (int iter = 0; iter < iters; iter++) {
        // assign step
        for (int r = 0; r < n; r++) {
            double best_d = 1e18;
            int best_k = 0;
            for (int k = 0; k < K; k++) {
                double dx = I.coords[r].first - cx[k];
                double dy = I.coords[r].second - cy[k];
                double d2 = dx * dx + dy * dy;
                if (d2 < best_d) {
                    best_d = d2;
                    best_k = k;
                }
            }
            assign[r] = best_k;
        }

        // update step
        std::vector<double> sumx(K, 0), sumy(K, 0);
        std::vector<int> cnt(K, 0);
        for (int r = 0; r < n; r++) {
            int k = assign[r];
            sumx[k] += I.coords[r].first;
            sumy[k] += I.coords[r].second;
            cnt[k]++;
        }
        for (int k = 0; k < K; k++) {
            if (cnt[k] > 0) {
                cx[k] = sumx[k] / cnt[k];
                cy[k] = sumy[k] / cnt[k];
            }
        }
    }

    return assign;
}


std::vector<int> build_route_greedy(
    const Instance& I,
    const std::vector<int>& reqs)
{
    std::vector<int> unpicked = reqs;           // requests not picked yet
    std::vector<int> active;                    // picked but not delivered
    std::vector<int> route;
    int cargo = 0;

    int last = 0;  // depot index

    auto demand_ok = [&](int r){ return cargo + I.demands[r] <= I.C; };

    while (!unpicked.empty() || !active.empty()) {

        int best_node = -1;
        int best_req  = -1;
        bool is_pick  = false;
        double best_d = 1e18;

        // candidate: pickups
        for (int r : unpicked) {
            if (demand_ok(r)) {
                int pickup_node = 1 + r;
                double d = I.dist[last][pickup_node];
                if (d < best_d) {
                    best_d = d;
                    best_req = r;
                    best_node = pickup_node;
                    is_pick = true;
                }
            }
        }

        // candidate: deliveries
        for (int r : active) {
            int deliver_node = 1 + I.n + r;
            double d = I.dist[last][deliver_node];
            if (d < best_d) {
                best_d = d;
                best_req = r;
                best_node = deliver_node;
                is_pick = false;
            }
        }

        if (best_node == -1) {
            // if no pickup was feasible, we MUST deliver something
            // pick nearest delivery
            double bd = 1e18;
            for (int r : active) {
                int dn = 1 + I.n + r;
                double d = I.dist[last][dn];
                if (d < bd) {
                    bd = d;
                    best_req = r;
                    best_node = dn;
                    is_pick = false;
                }
            }
        }

        // apply decision
        route.push_back(best_node);

        if (is_pick) {
            cargo += I.demands[best_req];
            active.push_back(best_req);
            unpicked.erase(std::remove(unpicked.begin(), unpicked.end(), best_req),
                           unpicked.end());
        } else {
            cargo -= I.demands[best_req];
            active.erase(std::remove(active.begin(), active.end(), best_req),
                         active.end());
        }

        last = best_node;
    }

    return route;
}


Solution DRC::construction(
    const Instance& I,
    double a,
    double sigma_factor,
    bool is_random)
{
    int K = I.nK;

    // 1. cluster requests spatially
    std::vector<int> cluster = kmeans_assign(I, K);

    // split into track lists
    std::vector<std::vector<int>> per_track(K);
    for (int r = 0; r < I.n; r++)
        per_track[cluster[r]].push_back(r);

    // 2. build route per track with greedy choice (pickup or delivery)
    gt::Matrix<int> routes;
    routes.reserve(K);
    for (int k = 0; k < K; k++) {
        auto rt = build_route_greedy(I, per_track[k]);
        routes.push_back(std::move(rt));
    }

    Solution sol;
    sol.routes = std::move(routes);
    return sol;
}

