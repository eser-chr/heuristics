/* Test of the lambda parameter in the random construction algorithm.
    Use of the instances till 2K requests.
*/

#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
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
    fs::path instance_path;
    int N;
    double alpha;
    int beam_width;
    double objective;
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

    out << "path,N,alpha,beam_width,objective\n";
    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.alpha << ","
            << r.beam_width << ","
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

    std::vector<int> Ns{50, 100, 200, 500, 1000, 2000};
    std::vector<int> beam_widths{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<double> alphas{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    std::vector<RES> res;
    res.reserve( Ns.size() * beam_widths.size() * alphas.size() * 20 );


    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_instance_paths(subdir);

        for (auto const& instance : instance_paths)
        {
            Instance I(instance);

            for (auto bw : beam_widths)
            {
                for (auto alpha : alphas)
                {
                    auto sol = BS::beam_search(I, alpha, bw);
                    auto f_sol = utils::objective(I, sol);
                    res.push_back(RES{instance, N, alpha, bw, f_sol});
                }
            }
        }
    }

    write_csv_results(base_output / "beam_tuning.csv", res);
}
