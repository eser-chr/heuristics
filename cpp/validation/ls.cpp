
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
    double objective;
    std::string method;
    std::string neighborhood;
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
            << r.objective << "\n";
    }
    out.close();
}

int main(int argc, char **argv)
{

    std::cout<<" I am starting with stepping method tuning"<<std::endl;
    auto [base_instances, base_output] = parse_paths(argc, argv);

    // std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/instances";
    // std::filesystem::path base_output = "/home/chris/Desktop/heuristics/results";

    std::vector<int> Ns{50, 100, 200, 500, 1000};
    // std::vector<int> Ns{50};

    std::vector<RES> res;
    std::map<std::string, StepFunction::Func> str_to_step{
        {"first", StepFunction::first_improvement},
        {"best", StepFunction::best_improvement},
        {"rand", StepFunction::random_step}};
        
    std::map<std::string, Neighborhood::NeighborhoodFactory> neighborhoods = {
        {"intra", [](const Instance &I, const Solution &s)
         { return std::make_unique<IntraRouteNeighborhood>(I, s); }},

        {"pair", [](const Instance &I, const Solution &s)
         { return std::make_unique<PairRelocateNeighborhood>(I, s); }},

        {"two-opt", [](const Instance &I, const Solution &s)
         { return std::make_unique<TwoOptNeighborhood>(I, s); }}
        
        };

    MaxIterations stopping_criterion(50);

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N);
        std::cout<<" I am starting instances with "<<N_str<<" requests\n\n";
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_some_instance_paths(subdir, 10);

        for (auto const &instance : instance_paths)
        {
            Instance I(instance);
            auto dr_sol = DC::construction(I);
            std::cout<<"-/"<<std::endl;

            for (auto const &[str_step, stepping_function] : str_to_step)
            {
                for (auto const &[str_neigh, neigh_factory] : neighborhoods)
                {
                    auto sol = LS::local_search(I, dr_sol, neigh_factory, stepping_function, stopping_criterion);
                    auto f_sol = utils::objective(I, sol);
                    res.push_back(RES{N, f_sol, str_step, str_neigh, instance});
                }
            }
        }
    }

    write_csv_results(base_output / "local_search_tuning.csv", res);
}
