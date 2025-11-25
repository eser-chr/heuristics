#include <chrono>
#include <iostream>
#include "utils.hpp"
#include "solvers.hpp"
#include "solution.hpp"
#include "instance.hpp"
#include "stopping_criteria.hpp"
#include "step_function.hpp"

int main()
{
    std::string path =
        "/home/chris/Desktop/heuristics/heuristics/instances/100/competition/instance61_nreq100_nveh2_gamma91.txt";
    // "/home/chris/Desktop/heuristics/heuristics/instances/50/train/instance1_nreq50_nveh2_gamma50.txt";
    // "/home/chris/Desktop/heuristics/heuristics/instances/100/train/instance1_nreq100_nveh2_gamma89.txt";
    // "/home/chris/Desktop/heuristics/heuristics/instances/1000/train/instance1_nreq1000_nveh20_gamma890.txt";

    
    try
    {
        auto t0 = std::chrono::high_resolution_clock::now();
        Instance I(path);
        auto t1 = std::chrono::high_resolution_clock::now();

        Solution sol_drc = DRC::construction(I, 1.0);
        double f_drc = utils::objective(I, sol_drc);
        auto t2 = std::chrono::high_resolution_clock::now();

        Neighborhood::NeighborhoodFactory neighborhoods = {
            [](const Instance &I, const Solution &s)
            { return std::make_unique<IntraRouteNeighborhood>(I, s); },
            [](const Instance &I, const Solution &s)
            { return std::make_unique<PairRelocateNeighborhood>(I, s); },
            [](const Instance &I, const Solution &s)
            { return std::make_unique<TwoOptNeighborhood>(I, s); }};

        MaxIterations stopping(500);

        Solution sol_ls = LS::local_search(
            I,
            sol_drc,
            neighborhoods,
            StepFunction::first_improvement,
            stopping);

        double f_ls = utils::objective(I, sol_ls);
        auto t3 = std::chrono::high_resolution_clock::now();

        Solution sol_beam = BS::beam_search(I, 1.0, 5);
        double f_beam = utils::objective(I, sol_beam);
        auto t4 = std::chrono::high_resolution_clock::now();

        Solution sol_vnd = VND::vnd(I, sol_drc, neighborhoods);
        double f_vnd = utils::objective(I, sol_vnd);
        auto t5 = std::chrono::high_resolution_clock::now();

        Solution sol_sa = SA::simulated_annealing(I, sol_drc, neighborhoods);
        double f_sa = utils::objective(I, sol_sa);
        auto t6 = std::chrono::high_resolution_clock::now();

        auto constructor = [&](const Instance &I)
        {
            return GRASP::randomized_constructor_simple(I, 0.8, 0.3, 20);
        };
        MaxIterations stopping_outer(50);
        MaxIterations stopping_local(2000);

        Solution sol_grasp = GRASP::grasp(
            I,
            constructor,
            neighborhoods,
            StepFunction::first_improvement,
            stopping_outer,
            stopping_local);

        double f_grasp = utils::objective(I, sol_grasp);
        auto t7 = std::chrono::high_resolution_clock::now();

        auto ms_parser       = std::chrono::duration<double, std::milli>(t1 - t0);
        auto ms_construction = std::chrono::duration<double, std::milli>(t2 - t1);
        auto ms_ls           = std::chrono::duration<double, std::milli>(t3 - t2);
        auto ms_bs           = std::chrono::duration<double, std::milli>(t4 - t3);
        auto ms_vnd          = std::chrono::duration<double, std::milli>(t5 - t4);
        auto ms_sa           = std::chrono::duration<double, std::milli>(t6 - t5);
        auto ms_grasp        = std::chrono::duration<double, std::milli>(t7 - t6);

        I.printme();
        std::cout << "\n Parser:       " << ms_parser.count()
                  << "\n Construction: " << ms_construction.count()
                  << "\n LS:           " << ms_ls.count()
                  << "\n Beam Search:  " << ms_bs.count()
                  << "\n VND:          " << ms_vnd.count()
                  << "\n SA:           " << ms_sa.count()
                  << "\n GRASP:        " << ms_grasp.count()
                  << "\n\n f(constr):    " << f_drc
                  << "\n f(BS):        " << f_beam
                  << "\n f(LS):        " << f_ls
                  << "\n f(VND):       " << f_vnd
                  << "\n f(SA):        " << f_sa
                  << "\n f(GRASP):     " << f_grasp
                  << std::endl;
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
