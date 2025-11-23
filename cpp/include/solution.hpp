#pragma once
#include <vector>

struct Solution
{
    std::vector<std::vector<int>> routes; // size = nK

    Solution() = default;   
    void write_solution(const std::string &path, const std::string &instance_name) const;
};