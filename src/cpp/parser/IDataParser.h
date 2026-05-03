#pragma once
#include "file.h"

class IDataParser {
public:
    virtual ~IDataParser() = default;

    virtual VerticesInfo parse_data_from_file(const std::filesystem::path& data_path) = 0;
};
