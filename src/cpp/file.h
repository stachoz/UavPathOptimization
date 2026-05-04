#pragma once

#include <charconv>
#include <ilcplex/ilocplex.h>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

struct VerticesInfo {
    int n; // depots
    int m; // demand points
    IloArray<IloNumArray> vertices_distances;
};

namespace file {
    inline int parse_line_int(const std::string &line) {
        int value = 0;
        const char *ptr = line.data();
        const char *end = line.data() + line.size();

        while (ptr < end && std::isspace(*ptr)) ptr++;

        auto [next_ptr, ec] = std::from_chars(ptr, end, value);
        if (ec != std::errc()) return -1;
        return value;
    }

    template<typename Con3D>
    void save_uav_paths(const IloCplex& cplex, const fs::path& output_file, const Con3D& con, int x, int y, int z) {
        std::ofstream file(output_file);

        if (!file.is_open()) {
            std::cerr << "Unable to create a file: " + output_file.string();
            return;
        }

        file << x << "\n";

        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {
                for (int k = 0; k < z; k++) {
                    file << cplex.getValue(con[i][j][k]) << " ";
                }
                file << "\n";
            }
            file << "\n";
        }
    }
}
