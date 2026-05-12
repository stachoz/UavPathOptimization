#include "UavPathOptimizationLogic.h"

void UavPathOptimizationLogic::run_solver(const VerticesInfo &vertices_info) const {
    try {
        const auto& env = CplexManager::getInstance().getEnv();
        const auto& dist = vertices_info.vertices_distances;

        IloInt n = vertices_info.n;         // Depots
        IloInt m = vertices_info.m;         // Demand Points
        IloInt num_nodes = n + m;           // Total nodes (5)
        IloInt u_count = 10;                 // UAVs

        // Demand point data (weight, volume, time windows)
        IloNumArray weight = vertices_info.weights;
        IloNumArray volume= vertices_info.volume;
        IloNumArray time_start = vertices_info.time_start;
        IloNumArray time_end = vertices_info.time_end;

        IloModel model(env);

        // dvars
        // x[u][i][j] - Boolean: UAV u travels from node i to j
        typedef IloArray<IloBoolVarArray> BoolVarMatrix;
        IloArray<BoolVarMatrix> x(env, u_count);
        for (int u = 0; u < u_count; u++) {
            x[u] = BoolVarMatrix(env, num_nodes);
            for (int i = 0; i < num_nodes; i++) {
                x[u][i] = IloBoolVarArray(env, num_nodes);
            }
        }

        // Continuous variables for Energy (e), Weight (w), Volume (v), Time (t)
        IloArray<IloNumVarArray> e(env, u_count);
        IloArray<IloNumVarArray> w(env, u_count);
        IloArray<IloNumVarArray> v(env, u_count);
        IloArray<IloNumVarArray> t(env, u_count);

        for (int u = 0; u < u_count; u++) {
            e[u] = IloNumVarArray(env, num_nodes, 0, power_max);
            w[u] = IloNumVarArray(env, num_nodes, 0, max_load);
            v[u] = IloNumVarArray(env, num_nodes, 0, max_volume);
            t[u] = IloNumVarArray(env, num_nodes, 0, IloInfinity);
        }

        // Objective
        IloExpr objExpr(env);
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    if (i == j) continue;
                    objExpr += dist[i][j] * x[u][i][j];
                }
            }
        }

        model.add(IloMinimize(env, objExpr));

        // Constraints
        // Initialization: UAV starts at the depot with its full spatial capacity
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < n; i++) {
                for (int j = n; j < num_nodes; j++) {
                    model.add(w[u][i] >= max_load - M * (1 - x[u][i][j]));
                    model.add(v[u][i] >= max_volume - M * (1 - x[u][i][j]));
                    model.add(e[u][i] >= power_max - M * (1 - x[u][i][j]));
                }
            }
        }

        // eq 2
        // Every demand point (n to n+m-1) served exactly once
        for (int j = n; j < num_nodes; j++) {
            IloExpr demand_conver(env);
            for (int u = 0; u < u_count; u++) {
                for (int i = 0; i < num_nodes; i++) {
                    if (i == j) continue;
                    demand_conver += x[u][i][j];
                }
            }
            model.add(demand_conver == 1);
        }

        // eq 3
        for (int u = 0; u < u_count; u++) {
            IloExpr launch_count(env), recovery_count(env);
            for (int i = 0; i < n; i++) {
                for (int j = n; j < num_nodes; j++) {
                    launch_count += x[u][i][j];
                    recovery_count += x[u][j][i];
                }
            }
            model.add(launch_count == recovery_count); // not enough because 0 == 0 satisfies the equation
            model.add(launch_count >= 1);
        }

        // eq 4
        for (int u = 0; u < u_count; u++) {
            for (int j = n; j < num_nodes; j++) {
                IloExpr demand_in(env), demand_out(env);
                for (int i = 0; i < num_nodes; i++) {
                    demand_in += x[u][i][j];
                    demand_out += x[u][j][i];
                }
                model.add(demand_in == demand_out);
            }
        }

        // eq 5
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    model.add(e[u][i] - power_rate * w[u][i] * dist[i][j] <= e[u][j] + M * (1 - x[u][i][j]));
                }
            }
        }

        // // eq 7
        // for (int u = 0; u < u_count; u++) {
        //     for (int i = 0; i < num_nodes; i++) {
        //         for (int j = 0; j < num_nodes; j++) {
        //             if (i == j) continue;
        //             model.add(w[u][i] - weight[i]<= w[u][j] + M * (1 - x[u][i][j]));
        //         }
        //     }
        // }

        // Corrected Load (Eq 7 & 8)
        // Equation 7 & 8: Corrected Load Depletion and Availability
        for (int u = 0; u < u_count; u++) {
            for (int j = n; j < num_nodes; j++) {
                IloExpr arrived_at_j(env);
                for (int i = 0; i < num_nodes; i++) { if (i != j) arrived_at_j += x[u][i][j]; }
                model.add(w[u][j] >= weight[j] * arrived_at_j);
            }

            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    if (i == j) continue;
                    // Only apply depletion if the UAV is moving to a demand point.
                    if (j >= n) {
                        model.add(w[u][j] <= w[u][i] - weight[i] + M * (1 - x[u][i][j]));
                    }
                }
            }
        }

        // eq 9
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    if (i == j) continue;
                    model.add(v[u][i] - volume[i] <= v[u][j] + M * (1 - x[u][i][j]));
                }
            }
        }

        // eq 11
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    if (i == j) continue;
                    model.add(t[u][i] + (dist[i][j] / speed + service_time) <= t[u][j] + M * (1 - x[u][i][j]));
                }
            }

            for (int i = 0; i < n; i++) {
                model.add(t[u][i] >= time_start[i]);
                model.add(t[u][i] <= time_end[i]);
            }
        }

        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::TimeLimit, 300);
        cplex.setParam(IloCplex::Param::MIP::Tolerances::RelObjDifference, 0.30);

        if (cplex.solve()) {
            env.out() << "Status: " << cplex.getStatus() << std::endl;
            env.out() << "Min Distance: " << cplex.getObjValue() << std::endl;
        }

        const fs::path uav_path_output = fs::path(PROJECT_ROOT_DIR) / "uav_path.txt";
        file::save_uav_paths(cplex, uav_path_output, x, u_count, num_nodes, num_nodes);

    } catch (IloException &e) {
        std::cerr << "Error: " << e << std::endl;
    }
}
