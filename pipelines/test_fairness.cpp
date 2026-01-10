
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <map>
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
    std::string fairness;
    std::string method;
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

    out << "path,N,method,duration,objective,fairness,min,max,mean,std\n";

    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.N << ","
            << r.method << ","
            << r.duration << ","
            << r.objective << ","
            << r.fairness << ","
            << r.min << ","
            << r.max << ","
            << r.mean_size_of_requests_per_track << ","
            << r.std_size_of_requests_per_track << "\n";
    }
    out.close();
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
    double s = std::accumulate(v.begin(), v.end(), 0.0, [m](double a, double x)
                               { return a + (x - m) * (x - m); });

    auto p = std::minmax_element(v.begin(), v.end());
    return stats{m, std::sqrt(s / (v.size() - 1)), *p.first, *p.second};
}

stats calc_stats(Solution const &sol)
{
    auto const &routes = sol.routes;

    std::vector<double> n_requests;
    n_requests.reserve(routes.size());

    for (size_t i = 0; i < routes.size(); ++i)
    {
        n_requests.push_back((double)routes[i].size() / 2);
    }
    return calc_stats(n_requests);
}

int main(int argc, char **argv)
{
    std::cout << "Enter MAIN A I compare dc rc " << std::endl;
    auto [base_instances, base_output, _] = parse_paths(argc, argv);

    std::vector<int> Ns{50, 100, 500, 1000};
    std::vector<std::string> fairness{"jain", "maxmin", "gini"};
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
        auto instance_paths = get_some_instance_paths(subdir, 5);

        for (auto fair_factor : fairness)
        {
            #pragma omp parallel for schedule(dynamic)
for (size_t idx = 0; idx < instance_paths.size(); ++idx)
{
    const auto& instance = instance_paths[idx];
    std::vector<RES> local_results;
    
    Solution dr_sol;
    Instance I(instance, fair_factor);
    
    {
        #pragma omp critical(cout)
        {
            std::cout << " -" << std::endl;
            std::cout << "dc";
        }
        
        Timer t;
        dr_sol = DC::construction(I);
        double exec_time = t.get_time();
        
        if (!dr_sol.is_solution_feasible(I))
        {
            #pragma omp critical(cout)
            std::cerr << "Solution is not feasible ERROR?";
        }
        
        double objective = utils::objective(I, dr_sol);
        auto stats = calc_stats(dr_sol);

        RES res;
        res.duration = exec_time;
        res.method = "DC ";
        res.fairness = fair_factor;
        res.instance_path = instance;
        res.objective = objective;
        res.N = N;
        res.mean_size_of_requests_per_track = stats.mean;
        res.std_size_of_requests_per_track = stats.std;
        res.min = stats.min;
        res.max = stats.max;
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
            std::cerr << "Solution is not feasible ERROR?";
        }
        
        double objective = utils::objective(I, rc_sol);
        auto stats = calc_stats(rc_sol);
        
        RES res;
        res.duration = exec_time;
        res.method = "RC";
        res.instance_path = instance;
        res.fairness = fair_factor;
        res.objective = objective;
        res.N = N;
        res.mean_size_of_requests_per_track = stats.mean;
        res.std_size_of_requests_per_track = stats.std;
        res.min = stats.min;
        res.max = stats.max;
        local_results.push_back(res);
    }
    
    {
        #pragma omp critical(cout)
        std::cout << " bs";
        
        Timer t;
        auto bs_sol = BS::beam_search(I, 1.0, 20);
        double exec_time = t.get_time();
        
        if (!bs_sol.is_solution_feasible(I))
        {
            #pragma omp critical(cout)
            std::cerr << "Solution is not feasible ERROR?";
        }
        
        double objective = utils::objective(I, bs_sol);
        auto stats = calc_stats(bs_sol);

        RES res;
        res.duration = exec_time;
        res.method = "BS";
        res.instance_path = instance;
        res.fairness = fair_factor;
        res.mean_size_of_requests_per_track = stats.mean;
        res.std_size_of_requests_per_track = stats.std;
        res.objective = objective;
        res.N = N;
        res.min = stats.min;
        res.max = stats.max;
        local_results.push_back(res);
    }
    
    {
        #pragma omp critical(cout)
        std::cout << " GA";
        
        Timer t;
        auto ga_sol = GA::genetic_algorithm(I, 15, 1, 20, 5);
        double exec_time = t.get_time();
        
        if (!ga_sol.is_solution_feasible(I))
        {
            #pragma omp critical(cout)
            std::cerr << "Solution is not feasible ERROR?";
        }
        
        double objective = utils::objective(I, ga_sol);
        auto stats = calc_stats(ga_sol);
        
        RES res;
        res.duration = exec_time;
        res.method = "GA";
        res.instance_path = instance;
        res.fairness = fair_factor;
        res.mean_size_of_requests_per_track = stats.mean;
        res.std_size_of_requests_per_track = stats.std;
        res.objective = objective;
        res.N = N;
        res.min = stats.min;
        res.max = stats.max;
        local_results.push_back(res);
    }
    
    #pragma omp critical(results)
    {
        all_res.insert(all_res.end(), local_results.begin(), local_results.end());
    }
}

        }
        std::cout << "\n";
    }

    write_csv_results(base_output / "fairness.csv", all_res);
}
