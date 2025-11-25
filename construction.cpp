// #include <vector>
// #include <algorithm>

// #include "utils.hpp"
// #include "solvers.hpp"

// void add_random_noise(const Instance &I, std::vector<double> noisy, const std::vector<double> &costs, double sigma_factor)
// {
//     static thread_local std::mt19937 rng(std::random_device{}());
//     double mean = std::accumulate(costs.begin(), costs.end(), 0.0) / costs.size();
//     double var = 0.0;
//     for (double x : costs)
//     {
//         double d = x - mean;
//         var += d * d;
//     }
//     var /= costs.size();
//     double stddev = std::sqrt(var);

//     double base = (stddev > 0.0 ? stddev : 1.0);
//     double sigma = sigma_factor * std::max(base, 1e-6);

//     std::normal_distribution<double> dist(0.0, sigma);
//     for (int i = 0; i < I.n; ++i)
//     {
//         noisy[i] += dist(rng);
//     }
// }

// gt::Matrix<int> get_ptrack_requests(int nK, const std::vector<int> requests)
// {
//     std::vector<std::vector<int>> per_track_requests(nK);
//     for (int t = 0; t < nK; ++t)
//     {
//         for (int idx = t; idx < (int)requests.size(); idx += nK)
//         {
//             per_track_requests[t].push_back(requests[idx]);
//         }
//     }
//     return per_track_requests;
// }

// // ---------------------------------------------
// // Noise + ranking
// // ---------------------------------------------
// std::vector<int> select_important(
//     const Instance& I,
//     double a,
//     double sigma_factor,
//     bool is_random)
// {
//     std::vector<double> costs = utils::calc_my_metric(I, a);
//     std::vector<double> noisy = costs;

//     if (is_random)
//         add_random_noise(I, noisy, costs, sigma_factor);

//     auto perm = numerical::argsort(noisy);
//     return std::vector<int>(perm.begin(), perm.begin() + I.gamma);
// }

// // ---------------------------------------------
// // Insert pickup, possibly triggering a delivery
// // ---------------------------------------------
// void handle_pickup(
//     const Instance& I,
//     int req,
//     std::vector<int>& route,
//     std::vector<int>& active,
//     int& cargo)
// {
//     auto less_dem = [&](int r1, int r2)
//     { return I.demands[r1] < I.demands[r2]; };

//     int dem = I.demands[req];

//     if (cargo + dem > I.C) {
//         int lightest =
//             *std::min_element(active.begin(), active.end(), less_dem);

//         active.erase(std::remove(active.begin(), active.end(), lightest),
//                      active.end());
//         route.push_back(1 + I.n + lightest);
//         cargo -= I.demands[lightest];
//     }

//     route.push_back(1 + req);
//     active.push_back(req);
//     cargo += dem;
// }

// // ---------------------------------------------
// // Flush remaining deliveries by nearest neighbor
// // ---------------------------------------------
// void flush_deliveries(
//     const Instance& I,
//     std::vector<int>& route,
//     std::vector<int>& active)
// {
//     int last = route.empty() ? 0 : route.back();

//     while (!active.empty()) {
//         int best_r = -1;
//         double best_d = 1e18;

//         for (int r : active) {
//             int deliver_node = 1 + I.n + r;
//             double d = I.dist[last][deliver_node];
//             if (d < best_d) {
//                 best_d = d;
//                 best_r = r;
//             }
//         }

//         int deliver_node = 1 + I.n + best_r;
//         route.push_back(deliver_node);
//         last = deliver_node;

//         active.erase(std::remove(active.begin(), active.end(), best_r),
//                      active.end());
//     }
// }

// // ---------------------------------------------
// // Build the route for a single track
// // ---------------------------------------------
// std::vector<int> build_track_route(
//     const Instance& I,
//     const std::vector<int>& reqs)
// {
//     std::vector<int> route;
//     std::vector<int> active;
//     int cargo = 0;

//     for (int req : reqs)
//         handle_pickup(I, req, route, active, cargo);

//     flush_deliveries(I, route, active);
//     return route;
// }

// // ---------------------------------------------
// // Ensure at least gamma requests served
// // ---------------------------------------------
// void ensure_gamma_served(
//     const Instance& I,
//     std::vector<int>& perm,
//     gt::Matrix<int>& routes)
// {
//     std::unordered_set<int> served;

//     for (const auto& r : routes)
//         for (int node : r)
//             if (int req = I.request_of_node[node]; req >= 0)
//                 served.insert(req);

//     if ((int)served.size() >= I.gamma)
//         return;

//     std::vector<int> missing;
//     missing.reserve(I.n);
//     for (int req : perm)
//         if (!served.count(req))
//             missing.push_back(req);

//     for (int req : missing) {
//         if ((int)served.size() >= I.gamma)
//             break;

//         int pickup = 1 + req;
//         int drop = 1 + I.n + req;

//         for (int k = 0; k < I.nK; ++k) {
//             auto new_r = routes[k];
//             new_r.push_back(pickup);
//             new_r.push_back(drop);

//             auto cargo_vec = utils::calc_route_cargo(I, new_r);
//             bool ok = true;
//             for (int c : cargo_vec)
//                 if (c < 0 || c > I.C) { ok = false; break; }

//             if (ok) {
//                 routes[k] = std::move(new_r);
//                 served.insert(req);
//                 break;
//             }
//         }
//     }
// }

// Solution DRC::construction(
//     const Instance& I,
//     double a,
//     double sigma_factor,
//     bool is_random)
// {
//     // 1. Select important requests
//     std::vector<double> costs = utils::calc_my_metric(I, a);
//     std::vector<double> noisy = costs;
//     if (is_random)
//         add_random_noise(I, noisy, costs, sigma_factor);
//     auto perm = numerical::argsort(noisy);

//     std::vector<int> important(perm.begin(), perm.begin() + I.gamma);
//     auto per_track_requests = get_ptrack_requests(I.nK, important);

//     // 2. Build routes for each track
//     gt::Matrix<int> routes;
//     routes.reserve(I.nK);

//     for (int track = 0; track < I.nK; ++track) {
//         auto route = build_track_route(I, per_track_requests[track]);
//         routes.push_back(std::move(route));
//     }

//     // 3. Guarantee gamma requests served
//     ensure_gamma_served(I, perm, routes);

//     // 4. Produce solution
//     Solution sol;
//     sol.routes = std::move(routes);
//     return sol;
// }

// // Solution DRC::construction(
// //     const Instance &I,
// //     double a,
// //     double sigma_factor,
// //     bool is_random)
// // {

// //     std::vector<double> costs = utils::calc_my_metric(I, a);
// //     std::vector<double> noisy = costs;

// //     if (is_random)
// //         add_random_noise(I, noisy, costs, sigma_factor);

// //     auto perm = numerical::argsort(noisy);
// //     std::vector<int> important(perm.begin(), perm.begin() + I.gamma); // Select first gamma requests to fullfill.
// //     auto per_track_requests = get_ptrack_requests(I.nK, important);

// //     gt::Matrix<int> routes;
// //     routes.reserve(I.nK);

// //     for (int track = 0; track < I.nK; ++track)
// //     {
// //         std::vector<int> route;
// //         std::vector<int> active;
// //         int cargo = 0;

// //         auto less_dem = [&](int r1, int r2)
// //         { return I.demands[r1] < I.demands[r2]; };
// //         auto greater_dem = [&](int r1, int r2)
// //         { return I.demands[r1] > I.demands[r2]; };

// //         for (int req : per_track_requests[track])
// //         {
// //             int pickup = 1 + req;
// //             int dem = I.demands[req];

// //             if (cargo + dem > I.C)
// //             {
// //                 // find the lightest request and deliver it i.e push deliver
// //                 // node to the route.
// //                 int lightest =
// //                     *std::min_element(active.begin(), active.end(), less_dem);

// //                 active.erase(std::remove(active.begin(), active.end(), lightest),
// //                              active.end());
// //                 route.push_back(1 + I.n + lightest);
// //                 cargo -= I.demands[lightest];
// //             }

// //             route.push_back(pickup);
// //             active.push_back(req);
// //             cargo += dem;
// //         }

// //         // drop remaining active in descending demand order
// //         // std::sort(active.begin(), active.end(), greater_dem);

// //         // Drop remaining in the nearest neighbor
// //         int last = route.empty() ? 0 : route.back(); // current end node

// //         while (!active.empty())
// //         {
// //             int best_r = -1;
// //             double best_d = 1e18;

// //             for (int r : active)
// //             {
// //                 int deliver_node = 1 + I.n + r;
// //                 double d = I.dist[last][deliver_node];
// //                 if (d < best_d)
// //                 {
// //                     best_d = d;
// //                     best_r = r;
// //                 }
// //             }

// //             // append best delivery
// //             int deliver_node = 1 + I.n + best_r;
// //             route.push_back(deliver_node);

// //             // update last
// //             last = deliver_node;

// //             // remove it from active
// //             active.erase(std::remove(active.begin(), active.end(), best_r),
// //                          active.end());
// //         }

// //         for (int r : active)
// //             route.push_back(1 + I.n + r);

// //         routes.push_back(std::move(route));
// //     }

// //     // Ensure we serve at least gamma requests
// //     std::unordered_set<int> served;
// //     for (const auto &r : routes)
// //     {
// //         for (int node : r)
// //         {
// //             int req = I.request_of_node[node];
// //             if (req >= 0)
// //                 served.insert(req);
// //         }
// //     }

// //     if ((int)served.size() < I.gamma)
// //     {
// //         std::vector<int> missing;
// //         missing.reserve(I.n);
// //         for (int req : perm)
// //         {
// //             if (!served.count(req))
// //                 missing.push_back(req);
// //         }

// //         for (int req : missing)
// //         {
// //             if ((int)served.size() >= I.gamma)
// //                 break;
// //             int pickup = 1 + req;
// //             int drop = 1 + I.n + req;
// //             for (int k = 0; k < I.nK; ++k)
// //             {
// //                 auto new_r = routes[k];
// //                 new_r.push_back(pickup);
// //                 new_r.push_back(drop);
// //                 auto cargo_vec = utils::calc_route_cargo(I, new_r);

// //                 bool ok = true;
// //                 for (int c : cargo_vec)
// //                 {
// //                     if (c < 0 || c > I.C)
// //                     {
// //                         ok = false;
// //                         break;
// //                     }
// //                 }
// //                 if (ok)
// //                 {
// //                     routes[k] = std::move(new_r);
// //                     served.insert(req);
// //                     break;
// //                 }
// //             }
// //         }
// //     }

// //     Solution sol;
// //     sol.routes = std::move(routes);
// //     return sol;
// // }

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>
#include "instance.hpp"
#include "solvers.hpp"
#include "solution.hpp"

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

// angle of request r around depot
double request_angle(const Instance &I, int r)
{
    // adapt if your coordinate fields differ
    double dx = I.coords[r].first - I.coords[0].first;
    double dy = I.coords[r].second - I.coords[0].second;
    return std::atan2(dy, dx);
}

// sort requests by angle and return permutation of indices
std::vector<int> angle_order(const Instance &I, const std::vector<int> &reqs)
{
    std::vector<std::pair<double, int>> tmp;
    tmp.reserve(reqs.size());
    for (int r : reqs)
        tmp.emplace_back(request_angle(I, r), r);

    std::sort(tmp.begin(), tmp.end(),
              [](const auto &a, const auto &b)
              { return a.first < b.first; });

    std::vector<int> perm;
    perm.reserve(reqs.size());
    for (auto &p : tmp)
        perm.push_back(p.second);
    return perm;
}

// split ordered requests into nK contiguous sectors
std::vector<std::vector<int>> sector_partition(
    const std::vector<int> &ordered_reqs,
    int nK)
{
    std::vector<std::vector<int>> per_track(nK);
    int m = (int)ordered_reqs.size();
    int base = m / nK;
    int extra = m % nK;

    int idx = 0;
    for (int k = 0; k < nK; ++k)
    {
        int take = base + (k < extra ? 1 : 0);
        if (take <= 0)
            continue;
        per_track[k].insert(per_track[k].end(),
                            ordered_reqs.begin() + idx,
                            ordered_reqs.begin() + idx + take);
        idx += take;
    }
    return per_track;
}

// handle one pickup, maybe forcing a delivery due to capacity
void handle_pickup(
    const Instance &I,
    int req,
    std::vector<int> &route,
    std::vector<int> &active,
    int &cargo)
{
    auto less_dem = [&](int r1, int r2)
    { return I.demands[r1] < I.demands[r2]; };

    int dem = I.demands[req];

    if (cargo + dem > I.C && !active.empty())
    {
        int lightest =
            *std::min_element(active.begin(), active.end(), less_dem);

        active.erase(std::remove(active.begin(), active.end(), lightest),
                     active.end());
        route.push_back(1 + I.n + lightest);
        cargo -= I.demands[lightest];
    }

    // if still infeasible, you can either skip or force; here we skip this req
    if (cargo + dem > I.C)
        return;

    route.push_back(1 + req); // pickup node index
    active.push_back(req);
    cargo += dem;
}

// flush all remaining active requests by nearest-neighbor deliveries
void flush_deliveries(
    const Instance &I,
    std::vector<int> &route,
    std::vector<int> &active)
{
    if (active.empty())
        return;

    int last = route.empty() ? 0 : route.back(); // assume 0 = depot

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

        int deliver_node = 1 + I.n + best_r;
        route.push_back(deliver_node);
        last = deliver_node;

        active.erase(std::remove(active.begin(), active.end(), best_r),
                     active.end());
    }
}

// build one track route from its assigned requests (sector)
std::vector<int> build_track_route(
    const Instance &I,
    const std::vector<int> &reqs)
{
    std::vector<int> route;
    std::vector<int> active;
    int cargo = 0;

    for (int req : reqs)
        handle_pickup(I, req, route, active, cargo);

    flush_deliveries(I, route, active);
    return route;
}

// -----------------------------------------------------------------------------
// DRC construction with sector-based assignment
// -----------------------------------------------------------------------------
Solution DRC::construction(
    const Instance &I,
    double a,
    double sigma_factor,
    bool is_random)
{
    // score all requests
    std::vector<double> costs = utils::calc_my_metric(I, a);
    std::vector<double> noisy = costs;
    if (is_random)
        add_random_noise(I, noisy, costs, sigma_factor);

    // rank by score
    auto perm = numerical::argsort(noisy);

    // keep top gamma (clamped)
    int gamma = std::min(I.gamma, I.n);
    std::vector<int> selected(perm.begin(), perm.begin() + gamma);

    // order selected by angle around depot
    auto angle_perm = angle_order(I, selected);

    // split into nK angular sectors
    auto per_track_requests = sector_partition(angle_perm, I.nK);

    // build routes per track
    gt::Matrix<int> routes;
    routes.reserve(I.nK);
    for (int k = 0; k < I.nK; ++k)
    {
        auto route = build_track_route(I, per_track_requests[k]);
        routes.push_back(std::move(route));
    }

    Solution sol;
    sol.routes = std::move(routes);
    return sol;
}
