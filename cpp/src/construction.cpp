/*
The idea of this construction heristic is to clusterize the requests into regions and then add them into routes.
Another possible idea is to use a convex hull for clusters. Maybe in Python.
*/

#include <vector>
#include <algorithm>
#include <random>
#include <limits>
#include "instance.hpp"
#include "solution.hpp"
#include "solvers.hpp"
#include "utils.hpp"

class ClusterCenters
{
public:
    using centers_t = std::vector<gt::Coords>;
    centers_t centers;

    ClusterCenters(const Instance &I) : centers(I.nK, {0.0, 0.0})
    {
        std::vector<int> indices(I.n);
        std::iota(indices.begin(), indices.end(), 0);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(indices.begin(), indices.end(), gen);

        for (int k = 0; k < I.nK; k++)
        {
            centers[k] = I.coords[indices[k]];
        }
    }

    // Update via an assignment. Calcs the point average of that clusters requests(pickup only!)
    void update_centers(const Instance &I, const std::vector<int> &assign)
    {

        centers.assign(I.nK, {0.0, 0.0});

        std::vector<int> count(I.nK, 0);

        for (int r = 0; r < I.n; r++)
        {
            int k = assign[r];
            centers[k].first += I.coords[r].first;
            centers[k].second += I.coords[r].second;
            count[k]++;
        }
        for (int k = 0; k < I.nK; k++)
        {
            if (count[k] > 0)
            {
                centers[k].first /= count[k];
                centers[k].second /= count[k];
            }
        }
    }
};

double calc_dist2(const gt::Coords &p1, const gt::Coords &p2)
{
    double dx = p1.first - p2.first;
    double dy = p1.second - p2.second;
    return dx * dx + dy * dy;
}

std::vector<int> balanced_assign(
    const Instance &I,
    const ClusterCenters &C,
    double target_load)
{

    std::vector<int> assign(I.n, -1);    // Assing request to cluster center
    std::vector<double> load(I.nK, 0.0); // Calc the load of each cluster

    for (int r = 0; r < I.n; r++)
    {
        double best_score{std::numeric_limits<double>::infinity()};
        int best_k = 0;

        for (int k = 0; k < I.nK; k++)
        {

            double dist2 = calc_dist2(I.coords[r], C.centers[k]);
            double load_after = load[k] + I.demands[r];
            double load_dev = std::abs(load_after - target_load);
            double score = dist2 + load_dev * load_dev;

            if (score < best_score)
            {
                best_score = score;
                best_k = k;
            }
        }

        assign[r] = best_k;
        load[best_k] += I.demands[r];
    }

    return assign;
}

std::vector<int> balanced_kmeans(
    const Instance &I,
    int iters = 20,
    int restarts = 20)
{
    double total_dem = 0;
    for (int d : I.demands)
        total_dem += d;
    double target_load = total_dem / I.nK;

    std::vector<int> best_assign(I.n);
    double best_score{std::numeric_limits<double>::infinity()};

    for (int s = 0; s < restarts; s++)
    {
        ClusterCenters C(I);
        std::vector<int> assign(I.n, 0);
        double sc = 0.0;

        // Convergence iteration, to the actual minimization of the problem i.e find balanced Kmeans
        for (int it = 0; it < iters; it++)
        {
            assign = balanced_assign(I, C, target_load);
            C.update_centers(I, assign);
        }

        for (int r = 0; r < I.n; r++)
        {
            int k = assign[r];
            sc = calc_dist2(I.coords[r], C.centers[k]);
        }

        if (sc < best_score)
        {
            best_score = sc;
            best_assign = assign;
        }
    }

    return best_assign;
}

std::vector<int> build_route_greedy(
    const Instance &I,
    const std::vector<int> &reqs)
{
    std::vector<int> unpicked = reqs;
    std::vector<int> active;
    std::vector<int> route;
    int cargo = 0;
    int last = 0; // depot

    auto cap_ok = [&](int r)
    { return cargo + I.demands[r] <= I.C; };

    while (!unpicked.empty() || !active.empty())
    {
        int best_node = -1, best_req = -1;
        bool pick = false;
        double best_d = 1e18;

        // pickups
        for (int r : unpicked)
        {
            if (!cap_ok(r))
                continue;
            int pn = 1 + r;
            double d = I.dist[last][pn];
            if (d < best_d)
            {
                best_d = d;
                best_node = pn;
                best_req = r;
                pick = true;
            }
        }

        // deliveries
        for (int r : active)
        {
            int dn = 1 + I.n + r;
            double d = I.dist[last][dn];
            if (d < best_d)
            {
                best_d = d;
                best_node = dn;
                best_req = r;
                pick = false;
            }
        }

        // Finish by delivering all of them
        if (best_node == -1)
        {
            double bd{std::numeric_limits<double>::infinity()};
            for (int r : active)
            {
                int dn = 1 + I.n + r;
                double d = I.dist[last][dn];
                if (d < bd)
                {
                    bd = d;
                    best_node = dn;
                    best_req = r;
                    pick = false;
                }
            }
        }

        route.push_back(best_node);

        // cargo logisitcs
        if (pick)
        {
            cargo += I.demands[best_req];
            active.push_back(best_req);
            unpicked.erase(std::remove(unpicked.begin(), unpicked.end(), best_req),
                           unpicked.end());
        }
        else
        {
            cargo -= I.demands[best_req];
            active.erase(std::remove(active.begin(), active.end(), best_req),
                         active.end());
        }

        last = best_node;
    }

    return route;
}

Solution DRC::construction(
    const Instance &I,
    double a,
    double sigma_factor,
    bool is_random)
{

    std::vector<int> assign = balanced_kmeans(I);
    gt::Matrix<int> per_track(I.nK); // per track requests-responibilities
    for (int r = 0; r < I.n; r++)
        per_track[assign[r]].push_back(r);

    gt::Matrix<int> routes;
    routes.reserve(I.nK);
    for (int k = 0; k < I.nK; k++)
    {
        auto route = build_route_greedy(I, per_track[k]);
        routes.push_back(std::move(route));
    }

    Solution sol;
    sol.routes = std::move(routes);
    return sol;
}
