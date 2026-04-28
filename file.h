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
    IloArray<IloNumArray> verticesDistances;
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

    inline VerticesInfo read_vertices_info(IloEnv env, const fs::path &filename) {
        VerticesInfo vertices_info;
        std::ifstream file(filename);
        std::string line;

        std::getline(file, line);
        vertices_info.n = parse_line_int(line);

        std::getline(file, line);
        vertices_info.m = parse_line_int(line);

        IloArray<IloNumArray> distMatrix(env);

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            IloNumArray row(env);
            const char *ptr = line.data();
            const char *end = line.data() + line.size();

            while (ptr < end) {
                while (ptr < end && std::isspace(*ptr)) ptr++;
                if (ptr == end) break;

                double val;
                auto [next_ptr, ec] = std::from_chars(ptr, end, val);
                if (ec == std::errc()) {
                    row.add(val);
                    ptr = next_ptr;
                }
                else {
                    break;
                }
            }

            if (row.getSize() > 0) {
                distMatrix.add(row);
            }
        }

        vertices_info.verticesDistances = distMatrix;
        return vertices_info;
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
