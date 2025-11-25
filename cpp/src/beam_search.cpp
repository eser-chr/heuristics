#include <vector>
#include <algorithm>
#include <limits>
#include "solvers.hpp"
#include "utils.hpp"

void flush_deliveries(
    const Instance &I,
    std::vector<int> &route,       // will be extended
    const std::vector<int> &active // remaining pickups (to deliver)
)
{
    // Copy active list because we need to erase from it locally
    std::vector<int> remaining = active;

    int last = route.empty() ? 0 : route.back();

    while (!remaining.empty())
    {
        int best_r = -1;
        double best_d = 1e18;

        for (int r : remaining)
        {
            int deliver_node = 1 + I.n + r;
            double d = I.dist[last][deliver_node];
            if (d < best_d)
            {
                best_d = d;
                best_r = r;
            }
        }

        // Append best delivery
        int deliver_node = 1 + I.n + best_r;
        route.push_back(deliver_node);

        // Update last
        last = deliver_node;

        // Remove delivered request
        remaining.erase(
            std::remove(remaining.begin(), remaining.end(), best_r),
            remaining.end());
    }
}



Solution BS::beam_search(const Instance &I, double a, int beam_width)
{
    std::vector<double> costs = utils::calc_my_metric(I, a);
    auto perm = numerical::argsort(costs);
    int gamma = std::min(I.gamma, I.n);
    std::vector<int> important(perm.begin(), perm.begin() + gamma);

    // per_track_requests
    std::vector<std::vector<int>> per_track_requests(I.nK);
    for (int t = 0; t < I.nK; ++t)
    {
        for (int idx = t; idx < (int)important.size(); idx += I.nK)
        {
            per_track_requests[t].push_back(important[idx]);
        }
    }

    std::vector<std::vector<int>> routes;
    routes.reserve(I.nK);

    for (int track = 0; track < I.nK; ++track)
    {
        std::vector<int> remaining = per_track_requests[track];

        std::vector<BS::BeamState> partial_routes;
        partial_routes.push_back(BS::BeamState{
            0.0,
            {},       // route
            0,        // cargo
            {},       // active
            remaining // remaining
        });

        for (size_t step = 0; step < remaining.size(); ++step)
        {
            std::vector<BS::BeamState> new_beam;

            for (const auto &st : partial_routes)
            {
                const auto &route = st.route;
                int cargo = st.cargo;
                const auto &active = st.active;
                const auto &rem = st.remaining;

                // 1) pick remaining requests
                for (int req : rem)
                {
                    if (cargo + I.demands[req] <= I.C)
                    {
                        int p = 1 + req;
                        std::vector<int> new_route = route;
                        new_route.push_back(p);

                        std::vector<int> new_active = active;
                        new_active.push_back(req);

                        std::vector<int> new_remaining;
                        new_remaining.reserve(rem.size() - 1);
                        for (int r : rem)
                        {
                            if (r != req)
                                new_remaining.push_back(r);
                        }

                        int last = route.empty() ? 0 : route.back();
                        double new_score = st.score + I.dist[last][p];

                        new_beam.push_back(BS::BeamState{
                            new_score,
                            std::move(new_route),
                            cargo + I.demands[req],
                            std::move(new_active),
                            std::move(new_remaining)});
                    }
                }

                // 2) drop any active request
                for (int req : active)
                {
                    int d = 1 + I.n + req;
                    std::vector<int> new_route = route;
                    new_route.push_back(d);
                    int new_cargo = cargo - I.demands[req];

                    std::vector<int> new_active;
                    new_active.reserve(active.size() - 1);
                    for (int r : active)
                    {
                        if (r != req)
                            new_active.push_back(r);
                    }

                    std::vector<int> new_remaining = rem; // unchanged

                    int last = route.empty() ? 0 : route.back();
                    double new_score = st.score + I.dist[last][d];

                    new_beam.push_back(BS::BeamState{
                        new_score,
                        std::move(new_route),
                        new_cargo,
                        std::move(new_active),
                        std::move(new_remaining)});
                }
            }

            if (new_beam.empty())
                break;

            std::sort(new_beam.begin(), new_beam.end(),
                      [](const BS::BeamState &a, const BS::BeamState &b)
                      {
                          return a.score < b.score;
                      });

            if ((int)new_beam.size() > beam_width)
                new_beam.resize(beam_width);

            partial_routes = std::move(new_beam);
        }

        // finalise each candidate by dropping all active
        double best_score = std::numeric_limits<double>::infinity();
        std::vector<int> best_route;

        for (const auto &st : partial_routes)
        {
            std::vector<int> final_route = st.route;
            flush_deliveries(I, final_route, st.active);
            int d = utils::route_distance(I, final_route);
            if (d < best_score)
            {
                best_score = d;
                best_route = std::move(final_route);
            }
        }

        routes.push_back(std::move(best_route));
    }

    Solution sol;
    sol.routes = std::move(routes);
    return sol;
}
