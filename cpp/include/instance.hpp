#pragma once
#include <string>
#include <vector>
#include <string>

class Instance
{
public:
    using Matrix = std::vector<std::vector<int>>;
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

    Instance(const std::string &nm,
             int n_, int nK_, int C_, int gamma_, double rho_,
             const std::vector<int> &demands_,
             const Matrix &dist_,
             const std::vector<int> &request_of_node_,
             const std::vector<int> &load_change_)
        : name(nm),
          n(n_), nK(nK_), C(C_), gamma(gamma_), rho(rho_),
          demands(demands_),
          dist(dist_),
          request_of_node(request_of_node_),
          load_change(load_change_)
    {
    }

    Instance(const std::string &path);
    void printme() const;

private:
    void load_from_file(const std::string &path);
};