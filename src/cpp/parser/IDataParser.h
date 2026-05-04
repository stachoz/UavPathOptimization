#pragma once
#include "file.h"

enum class DataType : uint8_t {
    DIST,
    COORDS,
    UNKNOWN
};

inline DataType depot_type_form_string(std::string_view type_str) {
    if (type_str == "dist") {
        return DataType::DIST;
    }

    if (type_str == "coords") {
        return DataType::COORDS;
    }

    return DataType::UNKNOWN;
}

enum class VertexType {
    DEPOT,
    DEMAND_POINT,
    UNKNOWN
};

inline VertexType vertex_type_from_string(std::string_view type_str) {
    if (type_str == "depot") {
        return VertexType::DEPOT;
    }
    if (type_str == "demand") {
        return VertexType::DEMAND_POINT;
    }

    return VertexType::UNKNOWN;
}

class IDataParser {
public:
    virtual ~IDataParser() = default;

    virtual VerticesInfo parse_data_from_file(const std::filesystem::path& data_path) = 0;
};
