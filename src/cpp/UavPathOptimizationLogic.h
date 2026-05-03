#pragma once
#include <utility>

#include "file.h"
#include "parser/DistancesParser.h"
#include "parser/IDataParser.h"

enum class DataType : uint8_t {
    DIST,
    COORDS,
    UNKNOWN
};

inline DataType from_string(std::string_view type_str) {
    if (type_str == "dist") {
        return DataType::DIST;
    }

    if (type_str == "coords") {
        return DataType::COORDS;
    }

    return DataType::UNKNOWN;
}

class UavPathOptimizationLogic {
public:
    UavPathOptimizationLogic(fs::path filename, DataType data_type) : filename(std::move(filename)) {
        switch (data_type) {
            case DataType::DIST:
                data_parser = std::make_unique<DistancesParser>();
                break;
            case DataType::COORDS:
                // TODO
                return;
            case DataType::UNKNOWN:
                // TODO
                return;
        }

    }

    void run() const {
        const VerticesInfo vertices_info = data_parser->parse_data_from_file(filename);
        run_solver(vertices_info);
    }
private:
    static void run_solver(const VerticesInfo& vertices_info);

    std::unique_ptr<IDataParser> data_parser;
    std::filesystem::path filename;
};