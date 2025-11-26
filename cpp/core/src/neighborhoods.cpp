#include <algorithm>
#include <numeric>
#include "neighborhoods.hpp"
#include "utils.hpp"
#include <iostream>
// Creates a vector of info for each request in a route.
std::vector<PickupDeliveryInfo>
pickup_delivery_positions(const Instance &I, const std::vector<int> &route)
{
    struct Pos
    {
        int p_idx = -1;
        int d_idx = -1;
    };
    std::unordered_map<int, Pos> pos;

    for (int idx = 0; idx < (int)route.size(); ++idx)
    {
        int node = route[idx];
        int req = I.request_of_node[node];
        if (req >= 0)
        {
            if (I.load_change[node] > 0)
                pos[req].p_idx = idx;
            else
                pos[req].d_idx = idx;
        }
    }

    std::vector<PickupDeliveryInfo> out;
    out.reserve(pos.size());
    for (const auto &kv : pos)
    {
        int req = kv.first;
        int p_idx = kv.second.p_idx;
        int d_idx = kv.second.d_idx;
        if (p_idx >= 0 && d_idx >= 0)
        {
            int pickup_node = 2 * req + 1;
            int delivery_node = 2 * req + 2;
            out.push_back(PickupDeliveryInfo{p_idx, d_idx, req, pickup_node, delivery_node});
        }
    }
    return out;
}

void IntraRouteNeighborhood::generate(std::vector<GenericMove> &moves) const
{
    moves.clear();
    for (int r = 0; r < (int)sol.routes.size(); ++r)
    {
        const auto &route = sol.routes[r];
        int m = (int)route.size();
        for (int k = 0; k < m; ++k)
        {
            for (int l = k + 1; l < m; ++l) // No for l<k+1 because of double counting.
            {
                moves.push_back(GenericMove{1, {r, k, l}});
            }
        }
    }
}

std::optional<GenericMove>
IntraRouteNeighborhood::generate_random(std::mt19937 &rng) const
{
    int R = (int)sol.routes.size();
    if (R == 0)
        return std::nullopt;

    std::uniform_int_distribution<int> route_dist(0, R - 1);

    for (int tries = 0; tries < this->MAX_TRIES_RANDOM; tries++)
    {
        // 1) Pick a random route
        int r = route_dist(rng);
        const auto &route = sol.routes[r];
        int m = (int)route.size();

        // Must have at least 2 positions
        if (m < 2)
            continue;

        std::uniform_int_distribution<int> idx_dist(0, m - 1);

        // 2) Pick random indices k < l
        int k = idx_dist(rng);
        int l = idx_dist(rng);
        if (k == l)
            continue;
        if (k > l)
            std::swap(k, l);

        // 3) Build the move
        GenericMove mvs{1, {r, k, l}};

        // 4) Validate
        if (is_valid(mvs))
            return mvs;
    }

    return std::nullopt;
}

bool IntraRouteNeighborhood::is_valid(const GenericMove &mov) const
{
    int r = mov.data[0];
    int k = mov.data[1];
    int l = mov.data[2];

    auto route = sol.routes[r];
    std::swap(route[k], route[l]);

    auto cargo = utils::calc_route_cargo(I, route);
    for (int c : cargo)
    {
        if (c >= I.C)
            return false;
    }
    return true;
}

double IntraRouteNeighborhood::calc_delta(const GenericMove &mov) const
{
    int r = mov.data[0];
    int k = mov.data[1];
    int l = mov.data[2];

    if (k == l)
        return 0.0;
    if (k > l)
        std::swap(k, l);

    const auto &route = sol.routes[r];
    const auto &dist = I.dist;

    int x = route[k];
    int y = route[l];

    int A = (k > 0) ? route[k - 1] : 0;
    int B = (k + 1 < (int)route.size()) ? route[k + 1] : 0;
    int C = (l > 0) ? route[l - 1] : 0;
    int D = (l + 1 < (int)route.size()) ? route[l + 1] : 0;

    int delta_d;

    if (l == k + 1)
    {
        delta_d =
            dist[A][y] + dist[y][x] + dist[x][D] -
            (dist[A][x] + dist[x][y] + dist[y][D]);
    }
    else
    {
        delta_d =
            dist[A][y] + dist[y][B] + dist[C][x] + dist[x][D] -
            (dist[A][x] + dist[x][B] + dist[C][y] + dist[y][D]);
    }

    double d_old = utils::route_distance(I, route);
    double d_new = d_old + delta_d;

    double S_old = sol.total_distance;
    double Q_old = sol.sum_of_squares;

    double S_new = S_old - d_old + d_new;
    double Q_new = Q_old - d_old * d_old + d_new * d_new;

    double J_old = (S_old * S_old) / (I.nK * Q_old);
    double J_new = (S_new * S_new) / (I.nK * Q_new);

    return delta_d + I.rho * (J_old - J_new);
}

Solution IntraRouteNeighborhood::apply(const GenericMove &mov) const
{
    int r = mov.data[0];
    int k = mov.data[1];
    int l = mov.data[2];

    Solution new_sol = sol;
    std::swap(new_sol.routes[r][k], new_sol.routes[r][l]);

    auto all_distances = utils::all_route_distances(I, new_sol);
    std::vector<double> sq_distances;
    sq_distances.resize(all_distances.size());
    std::transform(all_distances.begin(), all_distances.end(), sq_distances.begin(), [](auto val)
                   { return val * val; });
    new_sol.total_distance = std::accumulate(all_distances.begin(), all_distances.end(), 0.0);

    new_sol.sum_of_squares = std::accumulate(sq_distances.begin(), sq_distances.end(), 0.0);
    return new_sol;
}

// =====================================================================
// 2. PairRelocateNeighborhood
// =====================================================================

void PairRelocateNeighborhood::generate(std::vector<GenericMove> &moves) const
{
    moves.clear();
    int R = (int)sol.routes.size();

    for (int r_from = 0; r_from < R; ++r_from)
    {
        const auto &routeA = sol.routes[r_from];
        auto infos = pickup_delivery_positions(I, routeA);

        for (const auto &info : infos)
        {
            for (int r_to = 0; r_to < R; ++r_to)
            {
                if (r_to == r_from)
                    continue;

                int lenB = (int)sol.routes[r_to].size();
                for (int p_new = 0; p_new <= lenB; ++p_new)
                {
                    for (int d_new = p_new + 1; d_new <= lenB + 1; ++d_new)
                    {
                        moves.push_back(GenericMove{
                            2,
                            {r_from, info.p_idx, info.d_idx,
                             r_to, p_new, d_new,
                             info.req, info.pickup_node, info.delivery_node}});
                    }
                }
            }
        }
    }
}

std::optional<GenericMove>
PairRelocateNeighborhood::generate_random(std::mt19937 &rng) const
{
    int R = (int)sol.routes.size();
    if (R < 2)
        return std::nullopt;

    // 1) Pick r_from randomly
    std::uniform_int_distribution<int> route_dist(0, R - 1);

    for (int tries = 0; tries < this->MAX_TRIES_RANDOM; tries++)
    {
        int r_from = route_dist(rng);
        const auto &routeA = sol.routes[r_from];

        // Need at least one pickup-delivery pair
        auto infos = pickup_delivery_positions(I, routeA);
        if (infos.empty())
            continue;

        // 2) Choose a random pair (pickup_idx, delivery_idx)
        std::uniform_int_distribution<int> info_dist(0, (int)infos.size() - 1);
        const auto &info = infos[info_dist(rng)];

        // 3) Pick r_to randomly, different from r_from
        int r_to = r_from;
        if (R > 1)
        {
            do
            {
                r_to = route_dist(rng);
            } while (r_to == r_from);
        }

        const auto &routeB = sol.routes[r_to];
        int lenB = (int)routeB.size();
        if (lenB < 0)
            continue;

        // 4) Random insertion positions for pickup and delivery
        std::uniform_int_distribution<int> pos_dist(0, lenB + 1); // inclusive

        int p_new = pos_dist(rng);
        std::uniform_int_distribution<int> d_dist(p_new + 1, lenB + 2);
        int d_new = d_dist(rng);

        // 5) Build the candidate move
        GenericMove m{
            2,
            {r_from, info.p_idx, info.d_idx,
             r_to, p_new, d_new,
             info.req, info.pickup_node, info.delivery_node}};

        // 6) Validate it
        if (is_valid(m))
            return m;
    }

    return std::nullopt;
}

bool PairRelocateNeighborhood::is_valid(const GenericMove &mov) const
{
    int r_from = mov.data[0];
    int p_old = mov.data[1];
    int d_old = mov.data[2];
    int r_to = mov.data[3];
    int p_new = mov.data[4];
    int d_new = mov.data[5];
    int pnode = mov.data[7];
    int dnode = mov.data[8];

    auto routeA = sol.routes[r_from];
    auto routeB = sol.routes[r_to];

    // FIX 1: erase in correct order depending on indices
    if (p_old < d_old)
    {
        routeA.erase(routeA.begin() + d_old);
        routeA.erase(routeA.begin() + p_old);
    }
    else
    {
        routeA.erase(routeA.begin() + p_old);
        routeA.erase(routeA.begin() + d_old);
    }

    if (p_new < 0 || p_new > (int)routeB.size())
        return false;
    if (d_new < 0 || d_new > (int)routeB.size() + 1)
        return false;

    routeB.insert(routeB.begin() + p_new, pnode);
    // after inserting pickup, delivery index is in [p_new+1, size]
    if (d_new > (int)routeB.size())
        return false;
    routeB.insert(routeB.begin() + d_new, dnode);

    for (int c : utils::calc_route_cargo(I, routeA))
        if (c > I.C)
            return false;
    for (int c : utils::calc_route_cargo(I, routeB))
        if (c > I.C)
            return false;

    return true;
}

// double PairRelocateNeighborhood::calc_delta(const GenericMove &mov) const
// {
//     int r_from = mov.data[0];
//     int p_old = mov.data[1];
//     int d_old = mov.data[2];
//     int r_to = mov.data[3];
//     int p_new = mov.data[4];
//     int d_new = mov.data[5];
//     int pnode = mov.data[7];
//     int dnode = mov.data[8];

//     const auto &routeA_orig = sol.routes[r_from];
//     const auto &routeB_orig = sol.routes[r_to];

//     // old total distance of the two routes
//     double old_cost =
//         (double)utils::route_distance(I, routeA_orig) +
//         (double)utils::route_distance(I, routeB_orig);

//     // apply move to copies
//     auto routeA = routeA_orig;
//     auto routeB = routeB_orig;

//     if (p_old < d_old)
//     {
//         routeA.erase(routeA.begin() + d_old);
//         routeA.erase(routeA.begin() + p_old);
//     }
//     else
//     {
//         routeA.erase(routeA.begin() + p_old);
//         routeA.erase(routeA.begin() + d_old);
//     }

//     routeB.insert(routeB.begin() + p_new, pnode);
//     if (d_new > (int)routeB.size())
//     {
//         // if out of bound, treat as worst-case big delta
//         return std::numeric_limits<double>::infinity();
//     }
//     routeB.insert(routeB.begin() + d_new, dnode);

//     double new_cost =
//         (double)utils::route_distance(I, routeA) +
//         (double)utils::route_distance(I, routeB);

//     return new_cost - old_cost;
// }

Solution PairRelocateNeighborhood::apply(const GenericMove &mov) const
{
    int r_from = mov.data[0];
    int p_old = mov.data[1];
    int d_old = mov.data[2];
    int r_to = mov.data[3];
    int p_new = mov.data[4];
    int d_new = mov.data[5];
    int pnode = mov.data[7];
    int dnode = mov.data[8];

    Solution new_sol = sol;

    auto &routeA = new_sol.routes[r_from];
    auto &routeB = new_sol.routes[r_to];

    if (p_old < d_old)
    {
        routeA.erase(routeA.begin() + d_old);
        routeA.erase(routeA.begin() + p_old);
    }
    else
    {
        routeA.erase(routeA.begin() + p_old);
        routeA.erase(routeA.begin() + d_old);
    }

    routeB.insert(routeB.begin() + p_new, pnode);
    routeB.insert(routeB.begin() + d_new, dnode);

    auto all_distances = utils::all_route_distances(I, new_sol);
    std::vector<double> sq_distances;
    sq_distances.resize(all_distances.size());
    std::transform(all_distances.begin(), all_distances.end(), sq_distances.begin(), [](auto val)
                   { return val * val; });
    new_sol.total_distance = std::accumulate(all_distances.begin(), all_distances.end(), 0.0);

    new_sol.sum_of_squares = std::accumulate(sq_distances.begin(), sq_distances.end(), 0.0);

    return new_sol;
}
double PairRelocateNeighborhood::calc_delta(const GenericMove &mov) const
{
    int r_from = mov.data[0];
    int p_old  = mov.data[1];
    int d_old  = mov.data[2];
    int r_to   = mov.data[3];
    int p_new  = mov.data[4];
    int d_new  = mov.data[5];
    int pnode  = mov.data[7];
    int dnode  = mov.data[8];

    const auto &routeA_orig = sol.routes[r_from];
    const auto &routeB_orig = sol.routes[r_to];

    double dA_old = utils::route_distance(I, routeA_orig);
    double dB_old = utils::route_distance(I, routeB_orig);

    auto routeA = routeA_orig;
    auto routeB = routeB_orig;

    if (p_old < d_old)
    {
        routeA.erase(routeA.begin() + d_old);
        routeA.erase(routeA.begin() + p_old);
    }
    else
    {
        routeA.erase(routeA.begin() + p_old);
        routeA.erase(routeA.begin() + d_old);
    }

    routeB.insert(routeB.begin() + p_new, pnode);
    if (d_new > (int)routeB.size())
        return std::numeric_limits<double>::infinity();
    routeB.insert(routeB.begin() + d_new, dnode);

    double dA_new = utils::route_distance(I, routeA);
    double dB_new = utils::route_distance(I, routeB);

    double S_old = sol.total_distance;
    double Q_old = sol.sum_of_squares;

    double S_new = S_old - dA_old - dB_old + dA_new + dB_new;
    double Q_new = Q_old
                 - dA_old * dA_old + dA_new * dA_new
                 - dB_old * dB_old + dB_new * dB_new;

    double J_old = (S_old * S_old) / (I.nK * Q_old);
    double J_new = (S_new * S_new) / (I.nK * Q_new);

    double delta_d = (dA_new - dA_old) + (dB_new - dB_old);
    return delta_d + I.rho * (J_old - J_new);
}



// =====================================================================
// 3. TwoOptNeighborhood
// =====================================================================

void TwoOptNeighborhood::generate(std::vector<GenericMove> &moves) const
{
    moves.clear();
    for (int r = 0; r < (int)sol.routes.size(); ++r)
    {
        const auto &route = sol.routes[r];
        int m = (int)route.size();
        for (int i = 0; i < m - 2; ++i)
        {
            for (int j = i + 2; j < m; ++j)
            {
                moves.push_back(GenericMove{3, {r, i, j}});
            }
        }
    }
}

std::optional<GenericMove>
TwoOptNeighborhood::generate_random(std::mt19937 &rng) const
{
    if (sol.routes.empty())
        return std::nullopt;

    std::uniform_int_distribution<int> route_dist(0, sol.routes.size() - 1);
    int rid = route_dist(rng);

    const auto &r = sol.routes[rid];
    if (r.size() < 4)
        return std::nullopt;

    std::uniform_int_distribution<int> idx_dist(0, r.size() - 1);

    for (int t = 0; t < this->MAX_TRIES_RANDOM; t++)
    { // few random attempts
        int i = idx_dist(rng);
        int j = idx_dist(rng);
        if (i == j)
            continue;
        if (i > j)
            std::swap(i, j);

        GenericMove m{3, {rid, i, j}};

        if (is_valid(m))
            return m;
    }

    return std::nullopt;
}

bool TwoOptNeighborhood::is_valid(const GenericMove &mov) const
{
    int r = mov.data[0];
    int i = mov.data[1];
    int j = mov.data[2];

    auto route = sol.routes[r];
    std::reverse(route.begin() + i, route.begin() + j + 1);

    for (int c : utils::calc_route_cargo(I, route))
        if (c > I.C)
            return false;

    int nreq = I.n;
    std::vector<int> p(nreq, -1), d(nreq, -1);

    for (int idx = 0; idx < (int)route.size(); ++idx)
    {
        int node = route[idx];
        int req = I.request_of_node[node];
        if (req < 0)
            continue;

        if (I.load_change[node] > 0)
        {
            if (p[req] == -1)
                p[req] = idx;
        }
        else
        {
            if (d[req] == -1)
                d[req] = idx;
        }
    }

    for (int req = 0; req < nreq; ++req)
    {
        if (p[req] != -1 && d[req] != -1 && p[req] > d[req])
            return false;
    }

    return true;
}

double TwoOptNeighborhood::calc_delta(const GenericMove &mov) const
{
    int r = mov.data[0];
    int i = mov.data[1];
    int j = mov.data[2];

    const auto &route = sol.routes[r];
    const auto &dist = I.dist;

    int A = (i > 0) ? route[i - 1] : 0;
    int x = route[i];
    int y = route[j];
    int B = (j + 1 < (int)route.size()) ? route[j + 1] : 0;

    int removed = dist[A][x] + dist[y][B];
    int added = dist[A][y] + dist[x][B];

    double delta_d = added - removed;

    double d_old = utils::route_distance(I, route);
    double d_new = d_old + delta_d;

    double S_old = sol.total_distance;
    double Q_old = sol.sum_of_squares;

    double S_new = S_old - d_old + d_new;
    double Q_new = Q_old - d_old * d_old + d_new * d_new;

    double J_old = (S_old * S_old) / (I.nK * Q_old);
    double J_new = (S_new * S_new) / (I.nK * Q_new);

    return delta_d + I.rho * (J_old - J_new);
}

Solution TwoOptNeighborhood::apply(const GenericMove &mov) const
{
    int r = mov.data[0];
    int i = mov.data[1];
    int j = mov.data[2];

    Solution new_sol = sol;
    std::reverse(new_sol.routes[r].begin() + i, new_sol.routes[r].begin() + j + 1);

    auto all_distances = utils::all_route_distances(I, new_sol);
    std::vector<double> sq_distances;
    sq_distances.resize(all_distances.size());
    std::transform(all_distances.begin(), all_distances.end(), sq_distances.begin(), [](auto val)
                   { return val * val; });
    new_sol.total_distance = std::accumulate(all_distances.begin(), all_distances.end(), 0.0);

    new_sol.sum_of_squares = std::accumulate(sq_distances.begin(), sq_distances.end(), 0.0);

    return new_sol;
}
