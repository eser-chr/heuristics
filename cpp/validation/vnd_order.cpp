#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <random>
#include <map>
#include <algorithm>
#include "solvers.hpp"
#include "utils.hpp"
#include "path_utils.hpp"

struct RES
{
    int N;
    int permutation;
    double objective;
    fs::path instance_path;
};

void write_csv_results(const fs::path &output_path, const std::vector<RES> &results)
{
    std::ofstream out(output_path);
    if (!out.is_open())
    {

        std::cerr << "Error: cannot open results file: "
                  << output_path << std::endl;
        return;
    }

    out << "path,N,objective\n";

    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.objective
            << "\n";
    }
    out.close();
}

int main(int argc, char **argv)
{
    auto [base_instances, base_output] = parse_paths(argc, argv);

    // std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/instances";
    // std::filesystem::path base_output = "/home/chris/Desktop/heuristics/results";

    std::vector<int> Ns{50, 100, 200, 500};
    std::vector<RES> res;

    MaxIterations stopping_criterion(500);

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_some_instance_paths(subdir, 10);

        Neighborhood::NeighborhoodFactories neighs_in_order = {
            [](const Instance &I, const Solution &s)
            { return std::make_unique<TwoOptNeighborhood>(I, s); },
            [](const Instance &I, const Solution &s)
            { return std::make_unique<PairRelocateNeighborhood>(I, s); },
            [](const Instance &I, const Solution &s)
            { return std::make_unique<IntraRouteNeighborhood>(I, s); }};


        for (auto const &instance : instance_paths)
        {

            Instance I(instance);
            auto sol_dc = DC::construction(I);

            auto sol_cp = sol_dc;
            Solution sol_new = VND::vnd(I, sol_cp, neighs_in_order, StepFunction::first_improvement, stopping_criterion);
            double obj = utils::objective(I, sol_new);

            RES r;
            r.N = I.n;
            r.objective = obj;
            r.instance_path = instance;
            res.push_back(r);
        }
    }

    write_csv_results(base_output / "vnd_validation.csv", res);
}
