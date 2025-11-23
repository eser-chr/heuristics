#include <vector>
#include <algorithm>
#include "solvers.hpp"
#include "utils.hpp"

Solution BS::beam_search(const Instance &I, double a, int beam_width)
{
    int n = I.n;
    int nK = I.nK;

    std::vector<double> costs = utils::calc_my_metric(I, a);

    // argsort(costs)
    std::vector<int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);
    std::sort(perm.begin(), perm.end(),
              [&](int i, int j)
              { return costs[i] < costs[j]; });

    int gamma = std::min(I.gamma, I.n);
    std::vector<int> important(perm.begin(), perm.begin() + gamma);

    // per_track_requests
    std::vector<std::vector<int>> per_track_requests(nK);
    for (int t = 0; t < nK; ++t)
    {
        for (int idx = t; idx < (int)important.size(); idx += nK)
        {
            per_track_requests[t].push_back(important[idx]);
        }
    }

    std::vector<std::vector<int>> routes;
    routes.reserve(nK);

    for (int track = 0; track < nK; ++track)
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
                    int dem = I.demands[req];
                    if (cargo + dem <= I.C)
                    {
                        int p = 1 + req;
                        std::vector<int> new_route = route;
                        new_route.push_back(p);
                        int new_cargo = cargo + dem;

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
                            new_cargo,
                            std::move(new_active),
                            std::move(new_remaining)});
                    }
                }

                // 2) drop any active request
                for (int req : active)
                {
                    int d = 1 + n + req;
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
            int last = final_route.empty() ? 0 : final_route.back();

            auto active = st.active;
            std::sort(active.begin(), active.end(),
                      [&](int r1, int r2)
                      {
                          int d1 = I.dist[last][1 + n + r1];
                          int d2 = I.dist[last][1 + n + r2];
                          return d1 < d2;
                      });

            for (int req : active)
            {
                final_route.push_back(1 + n + req);
            }

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
