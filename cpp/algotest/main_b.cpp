
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <map>
#include "solvers.hpp"
#include "utils.hpp"
#include "path_utils.hpp"

struct RES
{
    int N;
    std::string method;
    double objective;
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

    out << "path,N,method,neighborhood,iterations,duration,objective\n";

    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.method << ","
            << r.N << ","
            << r.duration << ","
            << r.objective << "\n";
    }
    out.close();
}


int main(int argc, char **argv)
{
    std::cout << "Enter GRASP" << std::endl;
    auto [base_instances, base_output, idx] = parse_paths(argc, argv);

    std::vector<int> Ns{50, 100, 200, 500};

    std::vector<RES> all_res;

    std::vector<Neighborhood::NeighborhoodFactory> neighborhoods = {
        [](const Instance &I, const Solution &s)
        { return std::make_unique<IntraRouteNeighborhood>(I, s); },

        [](const Instance &I, const Solution &s)
        { return std::make_unique<PairRelocateNeighborhood>(I, s); },

        [](const Instance &I, const Solution &s)
        { return std::make_unique<TwoOptNeighborhood>(I, s); }};

    for (auto N : Ns)
    {
        ImprovementThreshold stopping_criterion((double)N/10);
        std::string N_str = std::to_string(N);
        std::cout << "Enter " << N_str << std::endl;
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_instance_paths(subdir);
        auto instance = instance_paths[idx];

        // for (auto const &instance : instance_paths)
        // {
        std::cout << " -" << std::endl;
        Solution dr_sol;
        Instance I(instance);

        { // GRASP

            Timer t;
            auto constructor = [&](const Instance &I)
            {
                return GRASP::randomized_constructor_simple(I, 1.0, 0.5);
            };
            MaxIterations stopping_outer(100);
            MaxIterations stopping_local(2000);

            auto sol_grasp = GRASP::grasp(
                I,
                constructor,
                neighborhoods,
                StepFunction::first_improvement,
                stopping_outer,
                stopping_local);

            double exec_time = t.get_time();
            double f_grasp = utils::objective(I, sol_grasp);

            RES res;
            res.duration = exec_time;
            res.method = "GRASP";
            res.instance_path = instance;
            res.objective = f_grasp;
            res.N = N;
            all_res.push_back(res);
        }
    }
    std::cout << "\n";
    // }
    auto output_name = "grasp_" + std::to_string(idx) + ".csv";
    write_csv_results(base_output / output_name, all_res);
}
