#include <fstream>
#include <iostream>
#include "solution.hpp"

void Solution::write_solution(const std::string &path, const std::string &instance_name) const
{
    std::cout<< "nK = "<<routes.size()<<std::endl;
    std::ofstream f(path);
    if (!f)
    {
        throw std::runtime_error("Could not open file for writing: " + path);
    }

    f << instance_name << "\n";

    for (const auto &route : routes)
    {
        if (route.empty())
        {
            f << "\n"; // blank line for empty route
            std::cout<<"One empty route"<<std::endl;
        }
        else
        {
            for (size_t i = 0; i < route.size(); ++i)
            {
                f << route[i];
                if (i + 1 < route.size())
                    f << " ";
            }
            f << "\n";
        }
    }
}