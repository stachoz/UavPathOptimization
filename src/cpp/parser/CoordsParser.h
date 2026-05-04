#pragma once
#include "IDataParser.h"
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <charconv>

#include "CPlexManager.h"

class CoordsParser : public IDataParser {
public:
    ~CoordsParser() override = default;

    VerticesInfo parse_data_from_file(const std::filesystem::path &data_path) override {
        VerticesInfo vertices_info {};
        std::ifstream file(data_path);

        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + data_path.string());
        }

        std::vector<std::pair<double, double>> coords {};

        std::string line;
        while (std::getline(file, line)) {
            if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
                continue;
            }

            const char *ptr = line.data();
            const char *end = line.data() + line.size();

            double x;
            auto [ptr_after_x, ec_x] = std::from_chars(ptr, end, x);
            if (ec_x != std::errc()) {
                throw std::runtime_error("Failed to parse X coordinate in file: " + data_path.string());
            }
            ptr = skip_to_next(ptr_after_x, end);

            double y;
            auto [ptr_after_y, ec_y] = std::from_chars(ptr, end, y);
            if (ec_y != std::errc()) {
                throw std::runtime_error("Failed to parse Y coordinate in file: " + data_path.string());
            }

            ptr = skip_to_next(ptr_after_y, end);

            if (ptr == end) {
                 throw std::runtime_error("Missing type information in file: " + data_path.string());
            }

            std::string_view type_str(ptr, end - ptr);
            VertexType type = vertex_type_from_string(type_str);

            if (type == VertexType::UNKNOWN) {
                throw std::runtime_error("Unknown DataType encountered: " + std::string(type_str));
            }
            if (type == VertexType::DEPOT) {
                ++vertices_info.n;
            }
            if (type == VertexType::DEMAND_POINT) {
                ++vertices_info.m;
            }

            coords.emplace_back(x, y);
        }

        vertices_info.vertices_distances = distances_from_coords(coords);
        return vertices_info;
    }

private:
    static IloArray<IloNumArray> distances_from_coords(const std::vector<std::pair<double, double>>& coords) {
        const auto& env = CplexManager::getInstance().getEnv();
        size_t n = coords.size();
        IloArray<IloNumArray> dist_matrix(env, n);

        for (size_t i = 0; i < n; ++i) {
            dist_matrix[i] = IloNumArray(env, n);
            dist_matrix[i][i] = 0.0;
        }

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const auto d = distance(coords[i], coords[j]);
                dist_matrix[i][j] = d;
                dist_matrix[j][i] = d;

                std::cout << d << " ";
            }
            std::cout << "\n";
        }

        return dist_matrix;
    }

    static double distance(const std::pair<double, double>& c1, const std::pair<double, double>& c2) {
        auto [x1, y1] = c1;
        auto [x2, y2] = c2;

        auto dx = x1 - x2;
        auto dy = y1 - y2;

        return std::sqrt(dx * dx + dy * dy);
    }

    static const char* skip_to_next(const char* ptr, const char* end, char sep = ';') {
        while (ptr < end && *ptr != sep) {
            ++ptr;
        }

        if (ptr < end && *ptr == sep) {
            ++ptr;
        }

        while (ptr < end && std::isspace(static_cast<unsigned char>(*ptr))) {
            ++ptr;
        }

        return ptr;
    }
};
