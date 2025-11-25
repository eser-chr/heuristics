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
            instances.push_back(entry);
        }
    }
    return instances;
}

struct RES
{
    fs::path instance_path;
    double alpha;
    double objective;
};

void write_results(const fs::path &output_path, const std::vector<RES> &results)
{
    std::ofstream out(output_path);
    if (!out.is_open())
    {

        std::cerr << "Error: cannot open results file: "
                  << output_path << std::endl;
        return;
    }

    out << "path,alpha,objective\n";
    for (const auto &r : results)
    {
        out << r.instance_path.string() << ","
            << r.alpha << ","
            << r.objective << "\n";
    }
    out.close();
}

void scan_per_n(const fs::path &folder, const fs::path &csv_output)
{

    std::cout << " Enter scan per n" << folder << std::endl;

    auto instance_paths = get_instance_paths(folder);
    std::vector<double> alphas{0.1, 0.2, 0.3, 0.6, 0.7, 0.8, 0.9, 1.0};
    std::vector<RES> results{};

    for (const auto &instance_path : instance_paths)
    {
        Instance I(instance_path);
        std::cout << "Instance: " << instance_path << "\n";
        std::cout << "n=" << I.n << " gamma=" << I.gamma << " nK=" << I.nK << "\n";

        for (const double a : alphas)
        {
            std::cout << " alpha = " << a << std::endl;
            auto sol = DC::construction(I); 
            auto f = utils::objective(I, sol);  // or here
            results.push_back({instance_path, a, f});
        }
    }

    write_results(csv_output, results);
}

int main()
{
    fs::path folder = "/home/chris/Desktop/heuristics/heuristics/instances";
    fs::path results_folder = "/home/chris/Desktop/heuristics/heuristics/results/hypertuning/alpha";

    std::cout << "Hello Chris \n";
    if (!fs::exists(results_folder))
        fs::create_directories(results_folder);
    std::cout << "Created_directories \n"
              << std::endl;

    for (const auto &entry : fs::directory_iterator(folder))
    {
        std::cout << "Main loop scan \n"
                  << std::endl;
        if (!entry.is_directory())
            continue;

        

        fs::path subdir = entry.path();
        if (subdir.filename() == "5000" || subdir.filename()=="10000")
            continue;
        auto instance_folder = subdir / "test";
        fs::path csv_output = results_folder / (subdir.filename().string() + ".csv");

        scan_per_n(instance_folder, csv_output);
    }
}
