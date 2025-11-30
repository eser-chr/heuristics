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
    double objective;
    double T;
    int iters;
    double alpha;
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

    out << "path,N,T,iters,alpha,objective\n";

    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.T << ","
            << r.iters << ","
            << r.alpha << ","
            << r.objective << "\n";
    }
    out.close();
}


int main(int argc, char **argv)
{
    auto [base_instances, base_output] = parse_paths(argc, argv);
    std::cout<<" I am starting with SA tuning"<<std::endl;

    // std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/instances";
    // std::filesystem::path base_output = "/home/chris/Desktop/heuristics/results";

    std::vector<int> Ns{50, 100, 200, 500, 1000};

    std::vector<double> alphas = {0.95, 0.97, 0.99};
    std::vector<double> T0s = {10.0, 50.0, 100.0};
    double Tmin = 1e-3;
    std::vector<int> Ls = {100}; // for stopping criterion

    std::vector<RES> res;

    Neighborhood::NeighborhoodFactory neigh = [](Instance const &I, Solution const &sol)
    { return std::make_unique<IntraRouteNeighborhood>(I, sol); };

    // In general SA can accept more than one neighborhood but for this case we used only one.
    std::vector<Neighborhood::NeighborhoodFactory> neighs{neigh};

    MaxIterations stopping_criterion(500);

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::cout << " I am starting instances with " << N_str << " requests\n\n";
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_some_instance_paths(subdir, 10);

        for (auto const &instance : instance_paths)
        {
            std::cout << "-/";
            Instance I(instance);

            for (auto iters : Ls)
            {
                MaxIterations stopping_criterion(iters);
                auto dr_sol = DC::construction(I);

                for (auto T_start : T0s)
                {
                    for (auto alpha : alphas)
                    {

                        auto sol = SA::simulated_annealing(I, dr_sol, neighs, T_start, Tmin, alpha, StepFunction::first_improvement, stopping_criterion);
                        auto f_sol = utils::objective(I, sol);
                        res.push_back(RES{N, f_sol, T_start, iters, alpha, instance});
                    }
                }
            }
            std::cout << "\n\n";
        }
    }

    write_csv_results(base_output / "sa.csv", res);
}
