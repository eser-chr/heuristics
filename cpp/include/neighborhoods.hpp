#pragma once
#include <vector>
#include <functional>
#include <memory>
#include "instance.hpp"
#include "solution.hpp"
#include "utils.hpp"

struct GenericMove
{
    int type;              // 1 = intra, 2 = pair-relocate, 3 = 2-opt
    std::vector<int> data; // payload
};

struct PickupDeliveryInfo
{
    int p_idx;
    int d_idx;
    int req;
    int pickup_node;
    int delivery_node;
};

std::vector<PickupDeliveryInfo>
pickup_delivery_positions(const Instance &I, const std::vector<int> &route);

class Neighborhood
{
public:
    using NeighborhoodFactory =
        std::vector<std::function<std::unique_ptr<Neighborhood>(const Instance &, const Solution &)>>;
    const Instance & I;
    Solution sol;
    double f;

    Neighborhood(const Instance &I_, const Solution &sol_)
        : I(I_), sol(sol_), f(utils::objective(I_, sol_)) {}

    virtual ~Neighborhood() = default;

    virtual void generate(std::vector<GenericMove> &moves) const = 0;
    virtual bool is_valid(const GenericMove &mov) const = 0;
    virtual double calc_delta(const GenericMove &mov) const = 0;
    virtual Solution apply(const GenericMove &mov) const = 0;
};


/*
This neighborhood represents intra route permutations. A move is a swap of two nodes. Ofc in the generic case one has 
to check if that permutation is valid  i.e the cargo does not exceed the C of the track.
*/
class IntraRouteNeighborhood : public Neighborhood
{
public:
    IntraRouteNeighborhood(const Instance &I_, const Solution &sol_)
        : Neighborhood(I_, sol_) {}
    void generate(std::vector<GenericMove> &moves) const override;
    bool is_valid(const GenericMove &mov) const override;
    double calc_delta(const GenericMove &mov) const override;
    Solution apply(const GenericMove &mov) const override;
};

/* Move one request from one track to another*/
class PairRelocateNeighborhood : public Neighborhood
{
public:
    PairRelocateNeighborhood(const Instance &I_, const Solution &sol_)
        : Neighborhood(I_, sol_) {}
    void generate(std::vector<GenericMove> &moves) const override;
    bool is_valid(const GenericMove &mov) const override;
    double calc_delta(const GenericMove &mov) const override;
    Solution apply(const GenericMove &mov) const override;
};


/* Flips the sequence intra route e.g 0,1,2,3,4,5
    2-opt at i=1, 4
    0, --4,3,2,1 --,5
    Avoids crossings in the path.
*/
class TwoOptNeighborhood : public Neighborhood
{
public:
    TwoOptNeighborhood(const Instance &I_, const Solution &sol_)
        : Neighborhood(I_, sol_) {}
    void generate(std::vector<GenericMove> &moves) const override;
    bool is_valid(const GenericMove &mov) const override;
    double calc_delta(const GenericMove &mov) const override;
    Solution apply(const GenericMove &mov) const override;
};