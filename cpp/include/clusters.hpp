#include <vector>
#include <algorithm>
#include <random>
#include <limits>

class ClusterCenters
{
public:
    using centers_t = std::vector<gt::Coords>;
    centers_t centers;

    ClusterCenters(const Instance &I);

    // Update via an assignment. Calcs the point average of that clusters requests(pickup only!)
    void update_centers(const Instance &I, const std::vector<int> &assign);
};

std::vector<int> balanced_assign(
    const Instance &I,
    const ClusterCenters &C,
    double target_load);

std::vector<int> balanced_kmeans(
    const Instance &I,
    int iters,
    int restarts);