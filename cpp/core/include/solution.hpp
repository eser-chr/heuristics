#pragma once
#include <vector>

struct Solution
{
    std::vector<std::vector<int>> routes; // size = nK
    double total_distance; 
    double sum_of_squares; // sum of dists[i]**2. For efficient delta eval

    Solution() = default;   
    void write_solution(const std::string &path, const std::string &instance_name) const;
};