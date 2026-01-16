#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <numeric>
#include <limits>
#include <omp.h>
#include <algorithm>
#include <cmath>

#include "solvers.hpp"
#include "structures.hpp"
#include "path_utils.hpp"

namespace fs = std::filesystem;

/* =======================
   Configuration structure
   ======================= */
struct Combo
{
    int k;
    int bw1;
    int bw2;
};

struct RaceConfig
{
    Combo cfg;
    std::vector<double> objectives; // one per instance
    bool active = true;
};

/* =======================
   Helper: build grid
   ======================= */

std::vector<Combo> build_configs(
    const std::vector<int> &k1s,
    const std::vector<int> &k2s,
    const std::vector<int> &beam_widths)
{
    std::vector<Combo> configs;
    for (int k1 : k1s)
        for (int k2 : k2s)
            for (int bw : beam_widths)
                configs.push_back({k1, k2, bw});
    return configs;
}

void write_results_csv(
    const std::filesystem::path &out_path,
    int N,
    const std::vector<RaceConfig> &race)
{
    static bool header_written = false;

    std::ofstream out(out_path, std::ios::app);
    if (!out)
        throw std::runtime_error("Cannot open CSV file");

    if (!header_written)
    {
        out << "N,k1,k2,beam_width,mean_objective,n_instances\n";
        header_written = true;
    }

    for (const auto &r : race)
    {
        if (!r.active)
            continue;

        const auto &obj = r.objectives;
        double mean =
            std::accumulate(obj.begin(), obj.end(), 0.0) / obj.size();

        out << N << ","
            << r.cfg.k << ","
            << r.cfg.bw1 << ","
            << r.cfg.bw2 << ","
            << mean << ","
            << obj.size() << "\n";
    }
}

/* =======================
   Helper: count active
   ======================= */
size_t count_active(const std::vector<RaceConfig> &race)
{
    size_t c = 0;
    for (const auto &r : race)
        if (r.active)
            ++c;
    return c;
}

/* One-sided Wilcoxon signed-rank test
   H1: x > y (x is worse than y)
*/
double wilcoxon_pvalue(
    const std::vector<double> &x,
    const std::vector<double> &y)
{
    struct Diff
    {
        double abs;
        int sign;
    };
    std::vector<Diff> diffs;

    for (size_t i = 0; i < x.size(); ++i)
    {
        double d = x[i] - y[i];
        if (d != 0.0)
            diffs.push_back({std::abs(d), d > 0 ? 1 : -1});
    }

    if (diffs.empty())
        return 1.0;

    std::sort(diffs.begin(), diffs.end(),
              [](auto &a, auto &b)
              { return a.abs < b.abs; });

    double Wplus = 0.0;
    for (size_t i = 0; i < diffs.size(); ++i)
        if (diffs[i].sign > 0)
            Wplus += i + 1;

    size_t n = diffs.size();
    double mu = n * (n + 1) / 4.0;
    double sigma = std::sqrt(n * (n + 1) * (2 * n + 1) / 24.0);

    double z = (Wplus - mu - 0.5) / sigma;

    // One-sided p-value (normal approximation)
    return 1.0 - 0.5 * (1.0 + std::erf(z / std::sqrt(2)));
}

size_t find_best_config(const std::vector<RaceConfig> &race)
{
    double best_mean = std::numeric_limits<double>::infinity();
    size_t best_idx = 0;

    for (size_t i = 0; i < race.size(); ++i)
    {
        if (!race[i].active)
            continue;

        double mean =
            std::accumulate(race[i].objectives.begin(),
                            race[i].objectives.end(), 0.0) /
            race[i].objectives.size();

        if (mean < best_mean)
        {
            best_mean = mean;
            best_idx = i;
        }
    }
    return best_idx;
}
void eliminate_configs_wilcoxon(
    std::vector<RaceConfig> &race,
    double alpha = 0.05)
{
    size_t best = find_best_config(race);

    for (size_t i = 0; i < race.size(); ++i)
    {
        if (!race[i].active || i == best)
            continue;

        const auto &best_obj = race[best].objectives;
        const auto &cur_obj = race[i].objectives;

        if (cur_obj.size() < 2)
            continue; // too early to test

        double p = wilcoxon_pvalue(cur_obj, best_obj);

        if (p < alpha)
            race[i].active = false;
    }
}

/* =======================
   Main
   ======================= */
int main(int argc, char **argv)
{
    auto [base_instances, base_output, _] = parse_paths(argc, argv);

    std::vector<int> Ns{50, 100, 200, 500, 1000};
    std::vector<int> ks{5, 10, 15, 20, 25};
    std::vector<int> bw1s{0, 1, 2, 3};
    std::vector<int> bw2s{3, 6, 9};

    // std::vector<int> Ns{50};
    // std::vector<int> ks{2,4,6};
    // std::vector<int> bw1s{5};
    // std::vector<int> bw2s{5};

    size_t n_instances = 30;
    for (int N : Ns)
    {
        std::cout << "\n=== F-Race for N = " << N << " ===\n";

        fs::path subdir = base_instances / std::to_string(N) / "train";
        auto instance_paths = get_some_instance_paths(subdir, n_instances);

        auto configs = build_configs(ks, bw1s, bw2s);

        std::vector<RaceConfig> race;
        for (const auto &c : configs)
            race.push_back({c, {}, true});

        size_t inst_idx = 0;
        for (; inst_idx < instance_paths.size(); ++inst_idx)
        {
            if (count_active(race) <= 1)
                break;

            const auto &instance = instance_paths[inst_idx];
            Instance I(instance, "jain");
            auto dr_sol = DC::construction(I);

            std::cout << "Instance " << inst_idx + 1
                      << " | Active configs: "
                      << count_active(race) << "\n";

#pragma omp parallel for schedule(dynamic)
            for (size_t i = 0; i < race.size(); ++i)
            {
                if (!race[i].active)
                    continue;

                const auto &cfg = race[i].cfg;

                auto ln_sol = LN::large_neighborhood(
                    I,
                    dr_sol,
                    cfg.k,
                    30, 
                    cfg.bw1,
                    cfg.bw2,
                    nullptr
                );

                double obj = utils::objective(I, ln_sol);

#pragma omp critical
                race[i].objectives.push_back(obj);
            }

            double alpha_eff = std::min(0.2, 0.05 + 0.01 * inst_idx);

            // Eliminate after at least 3 instances
            if (inst_idx > 2)
                eliminate_configs_wilcoxon(race, alpha_eff);
        }

        if (inst_idx >= n_instances - 1)
            std::cout << "all instances tested" << std::endl;

        std::cout << "Winner(s) for N = " << N << ":\n";
        for (const auto &r : race)
        {
            if (!r.active)
                continue;
            std::cout << " k=" << r.cfg.k
                      << " bw1=" << r.cfg.bw1
                      << " bw2=" << r.cfg.bw2
                      << " evaluations=" << inst_idx << "\n";
        }
        write_results_csv(base_output / "ln_frace_results.csv", N, race);
    }

    return 0;
}
