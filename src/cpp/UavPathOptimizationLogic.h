#pragma once
#include <utility>

#include "file.h"
#include "parser/CoordsParser.h"
#include "parser/DistancesParser.h"
#include "parser/IDataParser.h"


class UavPathOptimizationLogic {
public:
    UavPathOptimizationLogic(fs::path filename, DataType data_type) : filename(std::move(filename)) {
        switch (data_type) {
            case DataType::DIST:
                data_parser = std::make_unique<DistancesParser>();
                break;
            case DataType::COORDS:
                data_parser = std::make_unique<CoordsParser>();
                break;
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
    void run_solver(const VerticesInfo& vertices_info) const ;

    std::unique_ptr<IDataParser> data_parser;
    std::filesystem::path filename;

    // Alg constants
    IloNum power_max = 6000;
    const IloNum power_rate = 2;
    const IloNum max_load = 5;
    const IloNum max_volume = 10;
    const IloNum speed = 5;
    const IloNum service_time = 60;
    const IloNum M = 1000000.0;               /// Big M
};