#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <random>
#include <map>
#include <algorithm>
#include "solvers.hpp"
#include "utils.hpp"

namespace fs = std::filesystem;

auto get_instance_paths(const fs::path &folder)
{
    if (!fs::exists(folder))
        throw std::runtime_error("folder Does not exist " + folder.string());

    std::vector<fs::path> instances{};
    for (const auto &entry : fs::directory_iterator(folder))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".txt")
        {
            // std::cout << entry.path() << std::endl;
            instances.push_back(entry.path());
        }
    }
    return instances;
}

auto get_some_instance_paths(const fs::path &folder, int num_of_instances)
{
    auto paths = get_instance_paths(folder);
    if (num_of_instances >= paths.size())
    {
        return paths;
    }

    std::mt19937 rng(std::random_device{}());
    std::vector<int> indices(paths.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);

    std::vector<fs::path> to_return;
    to_return.reserve(num_of_instances);
    for (size_t i = 0; i < num_of_instances; i++)
        to_return.push_back(paths[indices[i]]);

    return to_return;
}

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

struct ParsedPaths
{
    fs::path base_instances;
    fs::path base_output;
};

ParsedPaths parse_paths(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: ./run <instances_path> <output_path>\n";
        std::exit(1);
    }

    fs::path instances = argv[1];
    fs::path output = argv[2];

    if (!fs::exists(instances) || !fs::is_directory(instances))
    {
        std::cerr << "Error: instances_path is not a valid directory\n";
        std::exit(1);
    }

    if (!fs::exists(output))
    {
        fs::create_directories(output);
    }

    return ParsedPaths{instances, output};
}

int main(int argc, char **argv)
{
    auto [base_instances, base_output] = parse_paths(argc, argv);

    // std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/instances";
    // std::filesystem::path base_output = "/home/chris/Desktop/heuristics/results";

    std::vector<int> Ns{50, 100, 200};

    std::vector<double> alphas = {0.95, 0.97, 0.99};
    std::vector<double> T0s = {10.0, 50.0, 100.0};
    double Tmin = 1e-3;
    std::vector<int> Ls = {10, 20, 50}; // for stopping criterion

    std::vector<RES> res;

    Neighborhood::NeighborhoodFactory neigh = [](Instance const &I, Solution const &sol)
    { return std::make_unique<IntraRouteNeighborhood>(I, sol); };

    // In general SA can accept more than one neighborhood but for this case we used only one.
    std::vector<Neighborhood::NeighborhoodFactory> neighs{neigh};

    MaxIterations stopping_criterion(500);

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_some_instance_paths(subdir, 10);

        for (auto const &instance : instance_paths)
        {

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
        }
    }

    write_csv_results(base_output / "local_search_tuning.csv", res);
}
