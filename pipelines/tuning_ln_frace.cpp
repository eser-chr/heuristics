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
    std::vector<double> objectives;
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

/* =======================
   CSV output
   ======================= */
void write_results_csv(
    const fs::path &out_path,
    int N,
    double alpha,
    bool stagnation,
    const std::vector<RaceConfig> &race)
{
    static bool header_written = false;

    std::ofstream out(out_path, std::ios::app);
    if (!out)
        throw std::runtime_error("Cannot open CSV file");

    if (!header_written)
    {
        out << "N,alpha,stagnation,k,bw1,bw2,mean_objective,n_instances\n";
        header_written = true;
    }

    for (const auto &r : race)
    {
        if (!r.active)
            continue;

        double mean = std::accumulate(
                          r.objectives.begin(),
                          r.objectives.end(),
                          0.0) /
                      r.objectives.size();

        out << N << ","
            << alpha << ","
            << stagnation << ","
            << r.cfg.k << ","
            << r.cfg.bw1 << ","
            << r.cfg.bw2 << ","
            << mean << ","
            << r.objectives.size() << "\n";
    }
}

/* =======================
   Helpers
   ======================= */
size_t count_active(const std::vector<RaceConfig> &race)
{
    size_t c = 0;
    for (const auto &r : race)
        if (r.active)
            ++c;
    return c;
}

/* =======================
   Wilcoxon signed-rank
   ======================= */
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
              [](const Diff &a, const Diff &b)
              { return a.abs < b.abs; });

    double Wplus = 0.0;
    for (size_t i = 0; i < diffs.size(); ++i)
        if (diffs[i].sign > 0)
            Wplus += i + 1;

    size_t n = diffs.size();
    double mu = n * (n + 1) / 4.0;
    double sigma = std::sqrt(n * (n + 1) * (2 * n + 1) / 24.0);

    double z = (Wplus - mu - 0.5) / sigma;
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

        double mean = std::accumulate(
                          race[i].objectives.begin(),
                          race[i].objectives.end(),
                          0.0) /
                      race[i].objectives.size();

        if (mean < best_mean)
        {
            best_mean = mean;
            best_idx = i;
        }
    }
    return best_idx;
}

/* =======================
   Elimination + p tracking
   ======================= */
double eliminate_configs_wilcoxon(
    std::vector<RaceConfig> &race,
    double alpha)
{
    size_t best = find_best_config(race);
    double min_p = 1.0;

    for (size_t i = 0; i < race.size(); ++i)
    {
        if (!race[i].active || i == best)
            continue;

        if (race[i].objectives.size() < 2)
            continue;

        double p = wilcoxon_pvalue(
            race[i].objectives,
            race[best].objectives);

        min_p = std::min(min_p, p);

        if (p < alpha)
            race[i].active = false;
    }

    return min_p;
}

/* =======================
   Main
   ======================= */
int main(int argc, char **argv)
{
    auto [base_instances, base_output, _] = parse_paths(argc, argv);

    // std::vector<int> Ns{50, 100, 200, 500};
    // std::vector<int> ks{2, 4, 6};
    // std::vector<int> bw1s{5, 10};
    // std::vector<int> bw2s{5, 10};

    std::vector<int> Ns{50, 100};
    std::vector<int> ks{2, 4};
    std::vector<int> bw1s{1, 2};
    std::vector<int> bw2s{3, 6};

    constexpr size_t max_instances = 15;
    constexpr int K = 3;

    std::vector<size_t> active_history;

    for (int N : Ns)
    {
        std::cout << "\n=== F-Race for N = " << N << " ===\n";

        fs::path subdir = base_instances / std::to_string(N) / "train";
        auto instance_paths = get_some_instance_paths(subdir, max_instances);

        auto configs = build_configs(ks, bw1s, bw2s);

        std::vector<RaceConfig> race;
        for (const auto &c : configs)
            race.push_back({c, {}, true});

        std::vector<double> best_p_history;
        bool stagnation = false;
        double alpha_eff = 0.1;

        size_t inst_idx = 0;
        for (; inst_idx < instance_paths.size(); ++inst_idx)
        {
            if (count_active(race) <= 1)
                break;

            Instance I(instance_paths[inst_idx], "jain");

            auto dr_sol = DC::construction(I);

            alpha_eff = std::max(0.25 - 0.02 * inst_idx, 0.1);

            std::cout << "Instance " << inst_idx + 1
                      << " | Active: " << count_active(race)
                      << " | alpha=" << alpha_eff << "\n";

#pragma omp parallel for schedule(dynamic)
            for (size_t i = 0; i < race.size(); ++i)
            {
                if (!race[i].active)
                    continue;

                auto sol = LN::large_neighborhood(I, dr_sol, race[i].cfg.k, 20, race[i].cfg.bw1, race[i].cfg.bw2, nullptr);

                double obj = utils::objective(I, sol);

#pragma omp critical
                race[i].objectives.push_back(obj);
            }

            if (inst_idx > 2)
            {
                eliminate_configs_wilcoxon(race, alpha_eff);

                active_history.push_back(count_active(race));

                if (active_history.size() >= K)
                {
                    bool no_progress = true;
                    for (size_t i = active_history.size() - K + 1;
                         i < active_history.size(); ++i)
                    {
                        if (active_history[i] < active_history[i - 1])
                        {
                            no_progress = false;
                            break;
                        }
                    }

                    if (no_progress)
                    {
                        std::cout << "Active-set stagnation detected.\n";
                        stagnation = true;
                        break;
                    }
                }
            }
        }

        // if (stagnation && count_active(race) > 1)
        // {
        //     size_t best = find_best_config(race);
        //     for (size_t i = 0; i < race.size(); ++i)
        //         race[i].active = (i == best);
        // }

        write_results_csv(
            base_output / "ln_frace_results.csv",
            N,
            alpha_eff,
            stagnation,
            race);
    }

    return 0;
}
