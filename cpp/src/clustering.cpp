#include <vector>
#include <algorithm>
#include <random>
#include <limits>
#include "instance.hpp"
#include "utils.hpp"

double calc_dist2(const gt::Coords &p1, const gt::Coords &p2)
{
    double dx = p1.first - p2.first;
    double dy = p1.second - p2.second;
    return dx * dx + dy * dy;
}

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