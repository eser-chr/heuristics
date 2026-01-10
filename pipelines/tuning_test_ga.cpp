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

void write_binary(const std::vector<std::vector<double>> &data, const std::string &filename)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Error: cannot open binary file: " << filename << std::endl;
        return;
    }

    size_t rows = data.size();
    file.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
    for (const auto &row : data)
    {
        size_t cols = row.size();
        file.write(reinterpret_cast<const char *>(&cols), sizeof(cols));
        file.write(reinterpret_cast<const char *>(row.data()), cols * sizeof(double));
    }
}

struct RES
{
    int N;
    int k1;
    int k2;
    int beam_width;
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

    out << "path,N,k1,k2,beam_width,duration,objective\n";

    for (const auto &r : results)
    {
        out << "\"" << r.instance_path.string() << "\","
            << r.N << ","
            << r.k1 << ","
            << r.k2 << ","
            << r.beam_width << ","
            << r.duration << ","
            << r.objective << "\n";
    }
}

struct combo
{
    int k1;
    int k2;
    int beam_width;
};

int main(int argc, char **argv)
{
    std::cout << "Start" << std::endl;
    auto [base_instances, base_output, _] = parse_paths(argc, argv);

    std::vector<int> Ns{50, 100, 200, 500, 1000, 2000};
    std::vector<RES> all_res;
    std::array<combo, 2> combos_to_be_tested{combo{15, 2, 5}, combo{15, 2, 10}};

    for (auto N : Ns)
    {
        std::string N_str = std::to_string(N); // Now with <string>
        std::cout << "Enter " << N_str << std::endl;
        fs::path subdir = base_instances / N_str / "test"; // Use fs::
        auto instance_paths = get_some_instance_paths(subdir, 5);

        for (const auto &instance : instance_paths)
        {
#pragma omp critical(cout)
            std::cout << " -" << std::endl;

            Instance I(instance, "jain");

            const size_t num_combos = combos_to_be_tested.size();
#pragma omp parallel for schedule(dynamic)
            for (size_t i = 0; i < num_combos; ++i)
            {
                auto const &combo = combos_to_be_tested[i];
                auto [k1, k2, beam_width] = combo;

#pragma omp critical(cout)
                std::cout << "Running GA: k1=" << k1 << " k2=" << k2
                          << " beam_width=" << beam_width << std::endl;

                Timer t;
                auto ga_sol = GA::genetic_algorithm(I, k1, k2, 30, beam_width);
                double exec_time = t.get_time();

                if (!ga_sol.is_solution_feasible(I))
                {
#pragma omp critical(cout)
                    std::cerr << "Solution is not feasible ERROR?" << std::endl;
                }

                double objective = utils::objective(I, ga_sol);

                RES res{N, k1, k2, beam_width, objective, exec_time, instance}; // Aggregate init

#pragma omp critical(results)
                all_res.push_back(res);
            }
        }
        std::cout << "\n";
    }

    write_csv_results(base_output / "tuning_ga.csv", all_res);
}
