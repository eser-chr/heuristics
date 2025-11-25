#pragma once
#include <string>
#include <vector>
#include <string>

class Instance
{
    using Matrix = std::vector<std::vector<double>>;
public:
    std::string name;
    int n;      // # requests
    int nK;     // # vehicles
    int C;      // vehicle capacity
    int gamma;  // min served requests
    double rho; // fairness weight

    std::vector<int> demands;         // size n
    Matrix dist;                      // (1 + 2n) x (1 + 2n)
    std::vector<int> request_of_node; // size (1 + 2n)
    std::vector<int> load_change;     // size (1 + 2n)
    std::vector<std::pair<double, double>> coords;

    Instance(const std::string &path);
    void printme() const;

private:
    void load_from_file(const std::string &path);
    bool checks();
};