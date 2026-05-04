#pragma once

#include "CPlexManager.h"
#include "IDataParser.h"

class DistancesParser : public IDataParser {
public:
    ~DistancesParser() override = default;

    VerticesInfo parse_data_from_file(const std::filesystem::path &filename) override {
        VerticesInfo vertices_info;
        std::ifstream file(filename);

        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename.string());
        }

        std::string line;

        std::getline(file, line);
        vertices_info.n = file::parse_line_int(line);

        std::getline(file, line);
        vertices_info.m = file::parse_line_int(line);

        const auto& iloEnv = CplexManager::getInstance().getEnv();
        IloArray<IloNumArray> dist_matrix(iloEnv);

        while (std::getline(file, line)) {
            if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
                continue;
            }

            IloNumArray row(iloEnv);
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
                    throw std::runtime_error("Failed to parse numeric value in file: " + filename.string());
                }
            }

            if (row.getSize() > 0) {
                dist_matrix.add(row);
            }
        }

        vertices_info.vertices_distances = dist_matrix;
        return vertices_info;
    }
};
