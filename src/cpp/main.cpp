#include <iostream>
#include "file.h"
#include "UavPathOptimizationLogic.h"

ILOSTLBEGIN

int main(int argc, char **argv) {

    if (argc == 3) {
        std::filesystem::path file_path = argv[1];
        const std::string data_type = argv[2];

        std::cout << file_path << std::endl;
        std::cout << data_type << std::endl;

        const auto logic = std::make_unique<UavPathOptimizationLogic>(file_path, depot_type_form_string(data_type));
        logic->run();
    }

    else {
        std::cout << "provide two argument: path to file and data type" << std::endl;
    }
    return 0;
}
