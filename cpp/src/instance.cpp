#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "instance.hpp"

void Instance::printme() const
{
    std::cout << name << " \n";
    std::cout << "---------" << "\n";
    std::cout << n << " " << nK << " " << C << " " << gamma << " " << rho << std::endl;
}

Instance::Instance(const std::string &path){
    load_from_file(path);
}

void Instance::load_from_file(const std::string &path)
    {
        std::ifstream f(path);
        if (!f)
            throw std::runtime_error("Could not open instance file: " + path);

        name = path;

        // read all lines
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(f, line))
            lines.push_back(line);

        // header
        {
            std::stringstream ss(lines[0]);
            ss >> n >> nK >> C >> gamma >> rho;
        }

        // find markers
        int idx_dem = -1, idx_loc = -1;
        for (int i = 0; i < (int)lines.size(); ++i)
        {
            if (lines[i].rfind("# demands", 0) == 0)
                idx_dem = i;
            if (lines[i].rfind("# request locations", 0) == 0)
                idx_loc = i;
        }
        if (idx_dem < 0 || idx_loc < 0)
            throw std::runtime_error("Bad instance file: missing markers");

        // parse demands
        {
            std::vector<int> tokens;
            for (int i = idx_dem + 1; i < idx_loc; i++)
            {
                std::stringstream ss(lines[i]);
                int x;
                while (ss >> x)
                    tokens.push_back(x);
            }
            if ((int)tokens.size() != n)
                throw std::runtime_error("Bad file: wrong number of demands");
            demands = tokens;
        }

        // number of nodes including depot
        int nV = 1 + 2 * n;

        // parse coordinates
        std::vector<std::pair<double, double>> coords;
        coords.reserve(nV);
        for (int i = idx_loc + 1; i <= idx_loc + nV; ++i)
        {
            std::stringstream ss(lines[i]);
            double x, y;
            ss >> x >> y;
            coords.emplace_back(x, y);
        }

        // build distance matrix
        dist.assign(nV, std::vector<int>(nV, 0));
        for (int u = 0; u < nV; u++)
        {
            for (int v = 0; v < nV; v++)
            {
                if (u == v)
                    continue;
                double dx = coords[u].first - coords[v].first;
                double dy = coords[u].second - coords[v].second;
                double d = std::sqrt(dx * dx + dy * dy);
                dist[u][v] = (int)std::ceil(d);
            }
        }

        // request_of_node + load_change
        request_of_node.assign(nV, -1);
        load_change.assign(nV, 0);

        for (int i = 0; i < n; i++)
        {
            int p = 1 + i;
            int d = 1 + n + i;

            request_of_node[p] = i;
            request_of_node[d] = i;

            load_change[p] = +demands[i];
            load_change[d] = -demands[i];
        }
    }