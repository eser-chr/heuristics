
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <map>
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

struct RES
{
    int N;
    double objective;
    std::string method;
    std::string neighborhood;
    double duration; // ms
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

    out << "path,N,method,neighborhood,objective\n";
    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.method << ","
            << r.neighborhood << ","
            << r.duration << ","
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
    std::cout << "Enter LS" << std::endl;
    auto [base_instances, base_output] = parse_paths(argc, argv);

    // std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/instances";
    // std::filesystem::path base_output = "/home/chris/Desktop/heuristics/results";

    std::vector<int> Ns{50, 100, 200, 500, 1000, 2000};

    std::vector<RES> res;
    std::map<std::string, StepFunction::Func> str_to_step{
        {"first", StepFunction::first_improvement},
        // {"best", StepFunction::best_improvement},
        {"rand", StepFunction::random_step}};

    std::map<std::string, Neighborhood::NeighborhoodFactory> neighborhoods = {
        {"intra", [](const Instance &I, const Solution &s)
         { return std::make_unique<IntraRouteNeighborhood>(I, s); }},

        {"pair", [](const Instance &I, const Solution &s)
         { return std::make_unique<PairRelocateNeighborhood>(I, s); }},

        {"two-opt", [](const Instance &I, const Solution &s)
         { return std::make_unique<TwoOptNeighborhood>(I, s); }}};

    // MaxIterations stopping_criterion(500);

    ImprovementThreshold stopping_criterion(10.0);

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::cout << "Enter " << N_str << std::endl;
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_instance_paths(subdir);

        for (auto const &instance : instance_paths)
        {
            std::cout << " -" << std::endl;
            Instance I(instance);
            auto dr_sol = DC::construction(I);

            for (auto const &[str_step, stepping_function] : str_to_step)
            {
                for (auto const &[str_neigh, neigh_factory] : neighborhoods)
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    auto sol = LS::local_search(I, dr_sol, neigh_factory, stepping_function, stopping_criterion);
                    auto t1 = std::chrono::high_resolution_clock::now();
                    auto f_sol = utils::objective(I, sol);
                    auto duration = std::chrono::duration<double, std::milli>(t1 - t0);
                    res.push_back(RES{N, f_sol, str_step, str_neigh, duration.count(), instance});
                }
            }
        }
        std::cout << "\n";
    }

    write_csv_results(base_output / "local_search_tuning.csv", res);
}
