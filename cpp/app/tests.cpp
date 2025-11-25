#include <chrono>
#include <iostream>
#include <filesystem>
#include "utils.hpp"
#include "solvers.hpp"
#include "solution.hpp"
#include "instance.hpp"
#include "stopping_criteria.hpp"
#include "step_function.hpp"

int main()
{

    // std::string path ="/home/chris/Desktop/heuristics/heuristics/instances/2000/competition/instance61_nreq2000_nveh40_gamma1829.txt";
    // std::string path ="/home/chris/Desktop/heuristics/heuristics/instances/1000/competition/instance61_nreq1000_nveh20_gamma879.txt";
    std::string instance_name = "instance61_nreq100_nveh2_gamma91";
    std::filesystem::path base_instances = "/home/chris/Desktop/heuristics/heuristics/instances/100/competition";
    std::filesystem::path instance_path = base_instances / (instance_name + ".txt");
    std::filesystem::path output = "/home/chris/Desktop/heuristics/heuristics/results/solutions/100";

    bool RUN_RANDOM = true;
    bool RUN_LS = true;
    bool RUN_BS = true;
    bool RUN_VND = true;
    bool RUN_SA = true;
    bool RUN_GRASP = false;
    bool RUN_METAGRASP = false;

    try
    {
        auto t0 = std::chrono::high_resolution_clock::now();
        Instance I(instance_path);
        auto t1 = std::chrono::high_resolution_clock::now();

        Solution sol_drc = DC::construction(I);
        sol_drc.write_solution(output / "dc.txt", instance_name);
        double f_drc = utils::objective(I, sol_drc);
        auto t2 = std::chrono::high_resolution_clock::now();

        Neighborhood::NeighborhoodFactories neighborhoods = {
            [](const Instance &I, const Solution &s)
            { return std::make_unique<IntraRouteNeighborhood>(I, s); },
            [](const Instance &I, const Solution &s)
            { return std::make_unique<PairRelocateNeighborhood>(I, s); },
            [](const Instance &I, const Solution &s)
            { return std::make_unique<TwoOptNeighborhood>(I, s); }};

        MaxIterations stopping(500);

        Solution sol_rc, sol_ls, sol_beam, sol_vnd, sol_sa, sol_grasp;
        double f_ls = 0, f_beam = 0, f_vnd = 0, f_sa = 0, f_grasp = 0, f_rc = 0, f_metagrasp = 0.0;

        auto t3 = t2, t4 = t2, t5 = t2, t6 = t2, t7 = t2, t8 = t2, t9=t2;

        if (RUN_RANDOM)
        {
            sol_rc = RC::construction(I,0.05 );
            f_rc = utils::objective(I, sol_rc);
            // sol_ls.write_solution(output/"ls.txt", instance_name);
            t8 = std::chrono::high_resolution_clock::now();
        }

        if (RUN_LS)
        {
            sol_ls = LS::local_search(
                I,
                sol_drc,
                neighborhoods[0],
                StepFunction::first_improvement,
                stopping);

            f_ls = utils::objective(I, sol_ls);
            sol_ls.write_solution(output / "ls.txt", instance_name);
            t3 = std::chrono::high_resolution_clock::now();
        }

        if (RUN_BS)
        {
            sol_beam = BS::beam_search(I, 1.0, 7);
            sol_beam.write_solution(output / "bs.txt", instance_name);
            f_beam = utils::objective(I, sol_beam);
            t4 = std::chrono::high_resolution_clock::now();
        }

        if (RUN_VND)
        {
            MaxIterations stopping(10000);
            sol_vnd = VND::vnd(I, sol_beam, neighborhoods, StepFunction::first_improvement, stopping);
            f_vnd = utils::objective(I, sol_vnd);
            t5 = std::chrono::high_resolution_clock::now();
        }

        if (RUN_SA)
        {
            MaxIterations stopping_criterion(10000);
            sol_sa = SA::simulated_annealing(I, sol_beam, neighborhoods, 1, 0.1, 0.995, StepFunction::random_step, stopping_criterion);
            f_sa = utils::objective(I, sol_sa);
            t6 = std::chrono::high_resolution_clock::now();
        }

        if (RUN_GRASP)
        {
            auto constructor = [&](const Instance &I)
            {
                // return RC::construction(I, 0.01);
                return GRASP::randomized_constructor_simple(I,1.0, 0.5);
            };
            MaxIterations stopping_outer(100);
            MaxIterations stopping_local(2000);

            sol_grasp = GRASP::grasp(
                I,
                constructor,
                neighborhoods,
                StepFunction::first_improvement,
                stopping_outer,
                stopping_local);

            f_grasp = utils::objective(I, sol_grasp);
            t7 = std::chrono::high_resolution_clock::now();
        }
        if (RUN_METAGRASP)
        {
            auto constructor = [&](const Instance &I)
            {
                return RC::construction(I, 0.01);
            };
            MaxIterations stopping_outer(100);
            MaxIterations stopping_local(2000);

            sol_grasp = GRASP::grasp(
                I,
                constructor,
                neighborhoods,
                StepFunction::first_improvement,
                stopping_outer,
                stopping_local);

            f_metagrasp = utils::objective(I, sol_grasp);
            t9 = std::chrono::high_resolution_clock::now();
        }

        auto ms_parser = std::chrono::duration<double, std::milli>(t1 - t0);
        auto ms_construction = std::chrono::duration<double, std::milli>(t2 - t1);
        auto ms_random = std::chrono::duration<double, std::milli>(t8 - t2);
        auto ms_ls = std::chrono::duration<double, std::milli>(t3 - t2);
        auto ms_bs = std::chrono::duration<double, std::milli>(t4 - t3);
        auto ms_vnd = std::chrono::duration<double, std::milli>(t5 - t4);
        auto ms_sa = std::chrono::duration<double, std::milli>(t6 - t5);
        auto ms_grasp = std::chrono::duration<double, std::milli>(t7 - t6);
        auto ms_metagrasp = std::chrono::duration<double, std::milli>(t9 - t7);

        I.printme();

        std::cout << "\n Parser:       " << ms_parser.count()
                  << "\n Construction: " << ms_construction.count();

        if (RUN_RANDOM)
            std::cout << "\n RANDOM:           " << ms_random.count();

        if (RUN_LS)
            std::cout << "\n LS:           " << ms_ls.count();
        if (RUN_BS)
            std::cout << "\n Beam Search:  " << ms_bs.count();
        if (RUN_VND)
            std::cout << "\n VND:          " << ms_vnd.count();
        if (RUN_SA)
            std::cout << "\n SA:           " << ms_sa.count();
        if (RUN_GRASP)
            std::cout << "\n GRASP:        " << ms_grasp.count();
        if (RUN_METAGRASP)
            std::cout << "\n META GRASP:        " << ms_metagrasp.count();

        std::cout << "\n\n f(constr):    " << f_drc;

        if (RUN_RANDOM)
            std::cout << "\n f(RANDOM):        " << f_rc;
        if (RUN_BS)
            std::cout << "\n f(BS):        " << f_beam;
        if (RUN_LS)
            std::cout << "\n f(LS):        " << f_ls;
        if (RUN_VND)
            std::cout << "\n f(VND):       " << f_vnd;
        if (RUN_SA)
            std::cout << "\n f(SA):        " << f_sa;
        if (RUN_GRASP)
            std::cout << "\n f(GRASP):     " << f_grasp;
        if (RUN_METAGRASP)
            std::cout << "\n f(METAGRASP):     " << f_metagrasp;

        std::cout << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "Unknown error" << std::endl;
    }
}
