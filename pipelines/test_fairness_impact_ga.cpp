#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <map>
#include <numeric>
#include <algorithm>
#include <cmath>

#include "solvers.hpp"
#include "structures.hpp"
#include "path_utils.hpp"

namespace fs = std::filesystem;

struct RES
{
    int N;
    double mean_size_of_requests_per_track;
    double std_size_of_requests_per_track;
    double min;
    double max;
    double objective;
    double duration; // ms
    std::string fairness;      // evaluated-as
    std::string fairness_opt;  // optimized-for
    std::string method;
    fs::path instance_path;
};

void write_csv_results(const fs::path &output_path,
                       const std::vector<RES> &results)
{
    std::ofstream out(output_path);
    if (!out.is_open())
    {
        std::cerr << "Error: cannot open results file: "
                  << output_path << std::endl;
        return;
    }

    out << "path,N,method,fairness_opt,fairness_eval,"
           "duration,objective,min,max,mean,std\n";

    for (const auto &r : results)
    {
        out << '"' << r.instance_path.string() << '"' << ","
            << r.N << ","
            << r.method << ","
            << r.fairness_opt << ","
            << r.fairness << ","
            << r.duration << ","
            << r.objective << ","
            << r.min << ","
            << r.max << ","
            << r.mean_size_of_requests_per_track << ","
            << r.std_size_of_requests_per_track
            << "\n";
    }
}

struct stats
{
    double mean;
    double std;
    double min;
    double max;
};

stats calc_stats(const std::vector<double> &v)
{
    double m = std::accumulate(v.begin(), v.end(), 0.0) / v.size();

    if (v.size() < 2)
        return {m, 0.0, m, m};

    double s = std::accumulate(
        v.begin(), v.end(), 0.0,
        [m](double a, double x) { return a + (x - m) * (x - m); });

    auto p = std::minmax_element(v.begin(), v.end());
    return {m, std::sqrt(s / (v.size() - 1)), *p.first, *p.second};
}

stats calc_stats(const Solution &sol)
{
    const auto &routes = sol.routes;

    std::vector<double> n_requests;
    n_requests.reserve(routes.size());

    for (const auto &r : routes)
        n_requests.push_back(static_cast<double>(r.size()) / 2.0);

    return calc_stats(n_requests);
}

int main(int argc, char **argv)
{
    auto [base_instances, base_output, _] = parse_paths(argc, argv);

    std::vector<int> Ns{50, 100, 500, 1000};
    std::vector<std::string> fairness{"jain", "maxmin", "gini"};
    std::vector<RES> all_res;

    for (int N : Ns)
    {
        fs::path subdir = base_instances / std::to_string(N) / "test";
        auto instance_paths = get_some_instance_paths(subdir, 5);

#pragma omp parallel for schedule(dynamic)
        for (size_t idx = 0; idx < instance_paths.size(); ++idx)
        {
            const auto &instance = instance_paths[idx];
            std::vector<RES> local_results;

            for (const auto &fair_opt : fairness)
            {
                Instance I_opt(instance, fair_opt);

                Timer t;
                auto ga_sol = GA::genetic_algorithm(I_opt, 10, 1, 20, 5);
                double exec_time = t.get_time();

                if (!ga_sol.is_solution_feasible(I_opt))
                {
#pragma omp critical(cout)
                    std::cerr << "Infeasible GA solution\n";
                }

                for (const auto &fair_eval : fairness)
                {
                    Instance I_eval(instance, fair_eval);

                    auto st = calc_stats(ga_sol);

                    RES res;
                    res.N = N;
                    res.method = "GA";
                    res.fairness_opt = fair_opt;
                    res.fairness = fair_eval;
                    res.instance_path = instance;
                    res.duration = exec_time;
                    res.objective = utils::objective(I_eval, ga_sol);
                    res.mean_size_of_requests_per_track = st.mean;
                    res.std_size_of_requests_per_track = st.std;
                    res.min = st.min;
                    res.max = st.max;

                    local_results.push_back(res);
                }
            }

#pragma omp critical(results)
            {
                all_res.insert(all_res.end(),
                               local_results.begin(),
                               local_results.end());
            }
        }
    }

    write_csv_results(base_output / "fairness.csv", all_res);
}
