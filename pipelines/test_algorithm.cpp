#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <map>
#include <omp.h>
#include "solvers.hpp"
#include "structures.hpp"
#include "path_utils.hpp"

namespace fs = std::filesystem;

struct RES
{
    int N;
    std::string method;
    double objective;
    double duration;
    fs::path instance_path;
};

void write_csv_results(const fs::path &output_path, const std::vector<RES> &results)
{
    std::ofstream out(output_path);
    if (!out.is_open())
    {
        std::cerr << "Error: cannot open results file: " << output_path << std::endl;
        return;
    }

    out << "path,N,method,duration,objective\n";

    for (const auto &r : results)
    {
        out << "\"" << r.instance_path.string() << "\","
            << r.N << ","
            << r.method << ","
            << r.duration << ","
            << r.objective << "\n";
    }
}

int main(int argc, char **argv)
{
    std::vector<int> const Ns{50, 100, 200, 500};
    int const N_of_instances = 30;

    std::cout << "Execute algorithm comparison" << std::endl;
    auto [base_instances, base_output, _] = parse_paths(argc, argv);

    std::vector<RES> all_res;
    std::vector<Neighborhood::NeighborhoodFactory> neighborhoods = {
        [](const Instance &I, const Solution &s)
        { return std::make_unique<IntraRouteNeighborhood>(I, s); },

        [](Instance const &I, Solution const &s)
        { return std::make_unique<RequestMove>(I, s); },

        [](const Instance &I, const Solution &s)
        { return std::make_unique<TwoOptNeighborhood>(I, s); }};

    for (auto N : Ns)
    {
        ImprovementThreshold stopping_criterion((double)N / 10);
        std::string N_str = std::to_string(N);
        std::cout << "Enter " << N_str << std::endl;
        std::filesystem::path subdir = base_instances / N_str / "test";
        auto instance_paths = get_some_instance_paths(subdir, N_of_instances);

        #pragma omp parallel for schedule(dynamic)
        for (size_t idx = 0; idx < instance_paths.size(); ++idx)
        {
            const auto& instance = instance_paths[idx];
            std::vector<RES> local_results;

            #pragma omp critical(cout)
            std::cout << " New Instance: " << instance << "\n --------------" << std::endl;

            Solution dr_sol;
            Instance I(instance, "jain");

            {
                #pragma omp critical(cout)
                std::cout << "dc";

                Timer t;
                dr_sol = DC::construction(I);
                double exec_time = t.get_time();

                if (!dr_sol.is_solution_feasible(I))
                {
                    #pragma omp critical(cout)
                    std::cerr << "ERROR: Solution not feasible for " << instance << std::endl;
                }

                double objective = utils::objective(I, dr_sol);

                RES res;
                res.duration = exec_time;
                res.method = "DC";
                res.instance_path = instance;
                res.objective = objective;
                res.N = N;
                local_results.push_back(res);
            }

            {
                #pragma omp critical(cout)
                std::cout << " rc";

                Timer t;
                auto rc_sol = RC::construction(I, 0.2);
                double exec_time = t.get_time();

                if (!rc_sol.is_solution_feasible(I))
                {
                    #pragma omp critical(cout)
                    std::cerr << "ERROR: Solution not feasible for " << instance << std::endl;
                }

                double objective = utils::objective(I, rc_sol);

                RES res;
                res.duration = exec_time;
                res.method = "RC";
                res.instance_path = instance;
                res.objective = objective;
                res.N = N;
                local_results.push_back(res);
            }

            {
                #pragma omp critical(cout)
                std::cout << " bs";

                Timer t;
                auto bs_sol = BS::beam_search(I, 0.9, 10);
                double exec_time = t.get_time();

                if (!bs_sol.is_solution_feasible(I))
                {
                    #pragma omp critical(cout)
                    std::cerr << "ERROR: Solution not feasible for " << instance << std::endl;
                }

                double objective = utils::objective(I, bs_sol);

                RES res;
                res.duration = exec_time;
                res.method = "BS";
                res.instance_path = instance;
                res.objective = objective;
                res.N = N;
                local_results.push_back(res);
            }

            {
                #pragma omp critical(cout)
                std::cout << " vnd";

                Timer t;
                auto sol_vnd = VND::vnd(I, dr_sol, neighborhoods, StepFunction::first_improvement, stopping_criterion);
                double exec_time = t.get_time();

                if (!sol_vnd.is_solution_feasible(I))
                {
                    #pragma omp critical(cout)
                    std::cerr << "ERROR: Solution not feasible for " << instance << std::endl;
                }

                double objective = utils::objective(I, sol_vnd);

                RES res;
                res.duration = exec_time;
                res.method = "VND";
                res.instance_path = instance;
                res.objective = objective;
                res.N = N;
                local_results.push_back(res);
            }

            {
                #pragma omp critical(cout)
                std::cout << " sa";

                Timer t;
                auto sol_sa = SA::simulated_annealing(I, dr_sol, neighborhoods,
                                                      100.0, 1e-3, 0.97, StepFunction::first_improvement, stopping_criterion);
                double exec_time = t.get_time();

                if (!sol_sa.is_solution_feasible(I))
                {
                    #pragma omp critical(cout)
                    std::cerr << "ERROR: Solution not feasible for " << instance << std::endl;
                }

                double objective = utils::objective(I, sol_sa);

                RES res;
                res.duration = exec_time;
                res.method = "SA";
                res.instance_path = instance;
                res.objective = objective;
                res.N = N;
                local_results.push_back(res);
            }

            {
                #pragma omp critical(cout)
                std::cout << " LN";

                Timer t;
                auto ln_sol = LN::large_neighborhood(I, dr_sol, 4, 2, 5, 5);
                double exec_time = t.get_time();

                if (!ln_sol.is_solution_feasible(I))
                {
                    #pragma omp critical(cout)
                    std::cerr << "ERROR: Solution not feasible for " << instance << std::endl;
                }

                double objective = utils::objective(I, ln_sol);

                RES res;
                res.duration = exec_time;
                res.method = "LN";
                res.instance_path = instance;
                res.objective = objective;
                res.N = N;
                local_results.push_back(res);
            }

            {
                #pragma omp critical(cout)
                std::cout << " GA";

                Timer t;
                auto ga_sol = GA::genetic_algorithm(I, 20, 0, 20, 10);
                double exec_time = t.get_time();

                if (!ga_sol.is_solution_feasible(I))
                {
                    #pragma omp critical(cout)
                    std::cerr << "ERROR: Solution not feasible for " << instance << std::endl;
                }

                double objective = utils::objective(I, ga_sol);

                RES res;
                res.duration = exec_time;
                res.method = "GA";
                res.instance_path = instance;
                res.objective = objective;
                res.N = N;
                local_results.push_back(res);
            }

            #pragma omp critical(cout)
            std::cout << "\n";

            #pragma omp critical(results)
            {
                all_res.insert(all_res.end(), local_results.begin(), local_results.end());
            }
        }
        std::cout << "\n";
    }

    write_csv_results(base_output / "algorithm_comparison.csv", all_res);
}
