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
    double initial;
    double small_rho;
    double high_rho;
    double small_gamma;
    double high_gamma;
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
            << r.initial << ","
            << r.small_rho << ","
            << r.high_rho << ","
            << r.small_gamma << ","
            << r.high_rho;
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
    std::vector<RES> res;

    double small_rho = 0.0;
    double high_rho = 10.0;

    MaxIterations stopping_criterion(500);

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_some_instance_paths(subdir, 10);

        for (auto const &instance : instance_paths)
        {

            Instance I(instance);
            double init_rho = I.rho;
            double init_gamma = I.gamma;
            RES tmp_res{};
            {
                auto sol_dc = DC::construction(I);
                tmp_res.initial = utils::objective(I, sol_dc);
            }
            {
                I.rho = small_rho;
                auto sol_dc = DC::construction(I);
                tmp_res.small_rho = utils::objective(I, sol_dc);
            }
            {
                I.rho = high_rho;
                auto sol_dc = DC::construction(I);
                tmp_res.high_rho = utils::objective(I, sol_dc);
            }
            {
                I.rho = init_rho;
                I.gamma = I.n;
                auto sol_dc = DC::construction(I);
                tmp_res.high_gamma = utils::objective(I, sol_dc);
            }
            {
                I.gamma = I.nK;
                auto sol_dc = DC::construction(I);
                tmp_res.small_gamma = utils::objective(I, sol_dc);
            }
            res.push_back(tmp_res);
        }
    }

    write_csv_results(base_output / "local_search_tuning.csv", res);
}
