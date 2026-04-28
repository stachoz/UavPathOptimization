#include <iostream>
#include "file.h"

ILOSTLBEGIN

int main() {
    IloEnv env;
    try {
        const VerticesInfo vertices_info = file::read_vertices_info(env, fs::path(PROJECT_ROOT_DIR) / "demandPointsInput.txt");
        const auto& dist = vertices_info.verticesDistances;

        IloInt n = vertices_info.n;                       // Depots
        IloInt m = vertices_info.m;                       // Demand Points
        IloInt u_count = 2;                 // UAVs
        IloInt num_nodes = n + m;           // Total nodes (5)
        IloNum M = 1000000.0;               // Big M

        IloNum power_max = 6000;
        IloNum power_rate = 2;
        IloNum max_load = 5;
        IloNum max_volume = 10;
        IloNum speed = 5;
        IloNum service_time = 60;

        // Demand point data (weight, volume, time windows)
        IloNumArray weight(env, m, 1.2, 0.8, 2.5);
        IloNumArray volume(env, m, 2.0, 1.5, 4.0);
        IloNumArray time_start(env, m, 0.0, 100.0, 200.0);
        IloNumArray time_end(env, m, 500.0, 600.0, 800.0);

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
                    objExpr += dist[i][j] * x[u][i][j];
                }
            }
        }

        model.add(IloMinimize(env, objExpr));

        // Constraints
        // initial energy at Depots (0 to n-1)
        for (int u = 0; u < u_count; u++)
            for (int i = 0; i < n; i++)
                model.add(e[u][i] == power_max);

        // Every demand point (n to n+m-1) served exactly once
        for (int j = n; j < num_nodes; j++) {
            IloExpr demandCover(env);
            for (int u = 0; u < u_count; u++)
                for (int i = 0; i < num_nodes; i++)
                    demandCover += x[u][i][j];
            model.add(demandCover == 1);
        }

        // Flow conservation
        for (int u = 0; u < u_count; u++) {
            for (int j = n; j < num_nodes; j++) {
                IloExpr inflow(env), outflow(env);
                for (int i = 0; i < num_nodes; i++) {
                    inflow += x[u][i][j];
                    outflow += x[u][j][i];
                }
                model.add(inflow - outflow == 0);
            }
        }

        // Energy constraints (Eq 5 & 6)
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    IloNum weight_i = (i >= n) ? weight[i - n] : 1.0;
                    model.add(e[u][i] - (power_rate * weight_i * dist[i][j]) <= e[u][j] + M * (1 - x[u][i][j]));
                }
            }
        }

        // Load and Volume constraints (Eq 7-10)
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = n; j < num_nodes; j++) {
                    model.add(w[u][i] - weight[j - n] <= w[u][j] + M * (1 - x[u][i][j]));
                    // Volume logic would follow a similar pattern if tracking decreases
                }
            }
        }

        // Time Windows (Eq 11 & 12)
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    model.add(
                        t[u][i] + (dist[i][j] / speed + service_time) * x[u][i][j] <= t[u][j] + M * (1 - x[u][i][j]));
                }
            }
            for (int i = n; i < num_nodes; i++) {
                model.add(t[u][i] >= time_start[i - n]);
                model.add(t[u][i] <= time_end[i - n]);
            }
        }

        // Solve
        IloCplex cplex(model);
        if (cplex.solve()) {
            env.out() << "Status: " << cplex.getStatus() << endl;
            env.out() << "Min Distance: " << cplex.getObjValue() << endl;
        }

        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < num_nodes; i++) {
                for (int j = 0; j < num_nodes; j++) {
                    std::cout << cplex.getValue(x[u][i][j]) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
    } catch (IloException &e) {
        cerr << "Error: " << e << endl;
    }
    env.end();
    return 0;
}
