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

    out << "path,initial,small_rho,high_rho,small_gamma,high_gamma\n";

    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.permutation << ","
            << r.objective;
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

    std::vector<int> Ns{50, 100, 200, 500};
    std::vector<RES> res;

    MaxIterations stopping_criterion(500);

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_some_instance_paths(subdir, 10);

        std::map<int, Neighborhood::NeighborhoodFactory> neighborhoods = {
            {0, [](const Instance &I, const Solution &s)
             { return std::make_unique<IntraRouteNeighborhood>(I, s); }},

            {1, [](const Instance &I, const Solution &s)
             { return std::make_unique<PairRelocateNeighborhood>(I, s); }},

            {2, [](const Instance &I, const Solution &s)
             { return std::make_unique<TwoOptNeighborhood>(I, s); }}};

        std::vector<int> neigh_id = {0, 1, 2};
        std::sort(neigh_id.begin(), neigh_id.end()); // ensure lexicographic perm

        for (auto const &instance : instance_paths)
        {

            Instance I(instance);
            auto sol_dc = DC::construction(I);

            do
            {
                // Build permutation code as integer, e.g. 0,1,2 -> 12
                int perm_code = neigh_id[0] * 100 + neigh_id[1] * 10 + neigh_id[2];

                // Build neighborhood objects in this order
                Neighborhood::NeighborhoodFactories local_neighs;
                local_neighs.reserve(neigh_id.size());
                for (int id : neigh_id)
                {
                    local_neighs.push_back(neighborhoods[id]);
                }

                // Run VND (or any local search you want)
                auto sol_cp = sol_dc;
                Solution sol_new = VND::vnd(I, sol_cp, local_neighs, StepFunction::first_improvement, stopping_criterion);
                double obj = utils::objective(I, sol_new);

                // Store result
                RES r;
                r.N = I.n;
                r.permutation = perm_code;
                r.objective = obj;
                r.instance_path = instance;
                res.push_back(r);

            } while (std::next_permutation(neigh_id.begin(), neigh_id.end()));
        }
    }

    write_csv_results(base_output / "local_search_tuning.csv", res);
}
