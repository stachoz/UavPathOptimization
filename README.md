# UAV Path Optimization

This project is a C++ application that utilizes **IBM ILOG CPLEX Studio** to solve an optimization problem for Unmanned Aerial Vehicle (UAV) routing. The model optimizes the paths of multiple UAVs traveling from depots to demand points, taking into account various constraints such as energy capacity, weight, volume, and service time windows.

The project also includes a Python script to visualize the nodes and the optimized UAV paths, either by applying Multidimensional Scaling (MDS) to a distance matrix or by plotting provided coordinates directly.

## Features

- **C++ CPLEX Model**: Solves a complex Mixed Integer Linear Programming (MILP) routing problem.
- **Cross-Platform CMake Build**: Configured to find and link CPLEX libraries automatically on both Windows (MSVC) and Linux.
- **Data Parsing**: Supports reading input data as either a distance matrix or a list of coordinates.
- **Visualization**: A Python script (`plot_mds.py`) to generate a 2D map of the depots, demand points, and the computed drone trajectories.

## Prerequisites

### For the C++ Optimization Model
- **C++ Compiler**: Must support C++17. (On Windows, **MSVC** via Visual Studio Build Tools is required for CPLEX compatibility; MinGW/GCC is not supported by CPLEX on Windows).
- **CMake**: Version 3.15 or newer.
- **IBM ILOG CPLEX Studio**: Installed on your system.

### For the Python Visualization Script
- **Python 3.x**
- Required packages: `numpy`, `matplotlib`, `scikit-learn`

You can install the Python dependencies using a virtual environment:
```bash
python -m venv venv
# Activate the environment (Windows)
venv\Scripts\activate
# Install dependencies
pip install numpy matplotlib scikit-learn
```

## Build Instructions

1. **Clone the repository** and navigate to the project root.
2. **Configure CMake**: 
   The build script attempts to automatically find CPLEX using the `CPLEX_STUDIO_DIR` environment variable. If it cannot find it, you must provide the path explicitly using the `CPLEX_ROOT` variable.

   ```bash
   mkdir build
   cd build
   cmake -DCPLEX_ROOT="C:/Program Files/IBM/ILOG/CPLEX_Studio2211" ..
   cmake --build .
   ```
   *(Note: Replace the path above with your actual CPLEX installation path).*

## Usage

### 1. Run the Optimization Model
Execute the compiled C++ program. It will read the input data (e.g., `demandPointsInput.txt` or `realTest.csv`), solve the routing problem using CPLEX, and output the computed routes to `uav_path.txt`.

### 2. Visualize the Results
Use the provided Python script to visualize the generated `uav_path.txt`. The script supports two input formats for the vertex data: `distances` (distance matrix) or `coords` (explicit X/Y coordinates).

**Option A: Distance Matrix Input**
If your input file contains a distance matrix, the script will use MDS to approximate the 2D layout:
```bash
python src/py/plot_mds.py test-data/demandPointsInput.txt distances --path_filename uav_path.txt
```

**Option B: Coordinate Input**
If your input file contains explicit coordinates (X; Y; type):
```bash
python src/py/plot_mds.py test-data/realTest.csv coords --path_filename uav_path.txt --skip_header 1
```

The script will save a high-resolution image named `mds_plot.png` in the project root directory.

## Project Structure

- `src/cpp/`: C++ source code for the CPLEX model and data parsers.
  - `main.cpp`: Entry point configuring and running the CPLEX solver.
  - `parser/`: Classes for parsing input data files (`DistancesParser.h`, `CoordsParser.h`).
  - `CPlexManager.h`: Meyer's Singleton managing the `IloEnv` CPLEX environment.
- `src/py/`: Python scripts.
  - `plot_mds.py`: Visualization script for drawing nodes and routes.
- `test-data/`: Example input data files (`.txt` and `.csv`).
- `CMakeLists.txt`: Build configuration.
