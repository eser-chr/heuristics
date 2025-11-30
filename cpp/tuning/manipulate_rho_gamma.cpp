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

    out << "path,N,initial,small_rho,high_rho,small_gamma,high_gamma\n";

    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.initial << ","
            << r.small_rho << ","
            << r.high_rho << ","
            << r.small_gamma << ","
            << r.high_rho
            <<"\n";
    }
    out.close();
}



int main(int argc, char **argv)
{
    auto [base_instances, base_output] = parse_paths(argc, argv);

    // std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/instances";
    // std::filesystem::path base_output = "/home/chris/Desktop/heuristics/results";

    std::vector<int> Ns{50, 100, 200, 500, 1000};
    std::vector<RES> res;

    double small_rho = 0.0;
    double high_rho = 1000.0;

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
            tmp_res.N = N;
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

    write_csv_results(base_output / "dc.csv", res);
}
