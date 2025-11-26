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
    double lamda;
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

    out << "path,N,lamda,objective\n";
    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.lamda << ","
            << r.objective << "\n";
    }
    out.close();
}

int main()
{
    std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/instances";
    std::filesystem::path base_output = "/home/chris/Desktop/heuristics/results";

    std::vector<int> Ns{50, 100, 200, 500, 1000, 2000};
    std::vector<double> lamdas{0.01, 0.03, 0.05, 0.07, 0.1, 0.15, 0.2};
    std::vector<RES> res;

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_instance_paths(subdir);

        for (auto instance : instance_paths)
        {
            Instance I(instance);

            for (auto lamda : lamdas)
            {
                auto sol = RC::construction(I, lamda);
                auto f_sol = utils::objective(I, sol);
                res.push_back(RES{instance, N, lamda, f_sol});
            }
        }
    }


    write_csv_results(base_output/"lamda_tuning.csv", res);
}
