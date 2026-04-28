#include <ilcplex/ilocplex.h>
#include <iostream>

ILOSTLBEGIN

int main() {
    IloEnv env;
    try {
        IloInt n = 2;                       // Depots
        IloInt m = 3;                       // Demand Points
        IloInt u_count = 2;                 // UAVs
        IloInt numNodes = n + m;            // Total nodes (5)
        IloNum M = 1000000.0;               // Big M

        // Distance Matrix
        IloArray<IloNumArray> dist(env, numNodes);
        dist[0] = IloNumArray(env, 5, 0.0, 10.0, 15.0, 20.0, 25.0);
        dist[1] = IloNumArray(env, 5, 10.0, 0.0, 12.0, 18.0, 22.0);
        dist[2] = IloNumArray(env, 5, 15.0, 12.0, 0.0, 5.0, 8.0);
        dist[3] = IloNumArray(env, 5, 20.0, 18.0, 5.0, 0.0, 6.0);
        dist[4] = IloNumArray(env, 5, 25.0, 22.0, 8.0, 6.0, 0.0);

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
            x[u] = BoolVarMatrix(env, numNodes);
            for (int i = 0; i < numNodes; i++) {
                x[u][i] = IloBoolVarArray(env, numNodes);
            }
        }

        // Continuous variables for Energy (e), Weight (w), Volume (v), Time (t)
        IloArray<IloNumVarArray> e(env, u_count);
        IloArray<IloNumVarArray> w(env, u_count);
        IloArray<IloNumVarArray> v(env, u_count);
        IloArray<IloNumVarArray> t(env, u_count);

        for (int u = 0; u < u_count; u++) {
            e[u] = IloNumVarArray(env, numNodes, 0, power_max);
            w[u] = IloNumVarArray(env, numNodes, 0, max_load);
            v[u] = IloNumVarArray(env, numNodes, 0, max_volume);
            t[u] = IloNumVarArray(env, numNodes, 0, IloInfinity);
        }

        // Objective
        IloExpr objExpr(env);
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < numNodes; i++) {
                for (int j = 0; j < numNodes; j++) {
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
        for (int j = n; j < numNodes; j++) {
            IloExpr demandCover(env);
            for (int u = 0; u < u_count; u++)
                for (int i = 0; i < numNodes; i++)
                    demandCover += x[u][i][j];
            model.add(demandCover == 1);
        }

        // Flow conservation
        for (int u = 0; u < u_count; u++) {
            for (int j = n; j < numNodes; j++) {
                IloExpr inflow(env), outflow(env);
                for (int i = 0; i < numNodes; i++) {
                    inflow += x[u][i][j];
                    outflow += x[u][j][i];
                }
                model.add(inflow - outflow == 0);
            }
        }

        // Energy constraints (Eq 5 & 6)
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < numNodes; i++) {
                for (int j = 0; j < numNodes; j++) {
                    IloNum weight_i = (i >= n) ? weight[i - n] : 1.0;
                    model.add(e[u][i] - (power_rate * weight_i * dist[i][j]) <= e[u][j] + M * (1 - x[u][i][j]));
                }
            }
        }

        // Load and Volume constraints (Eq 7-10)
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < numNodes; i++) {
                for (int j = n; j < numNodes; j++) {
                    model.add(w[u][i] - weight[j - n] <= w[u][j] + M * (1 - x[u][i][j]));
                    // Volume logic would follow a similar pattern if tracking decreases
                }
            }
        }

        // Time Windows (Eq 11 & 12)
        for (int u = 0; u < u_count; u++) {
            for (int i = 0; i < numNodes; i++) {
                for (int j = 0; j < numNodes; j++) {
                    model.add(
                        t[u][i] + (dist[i][j] / speed + service_time) * x[u][i][j] <= t[u][j] + M * (1 - x[u][i][j]));
                }
            }
            for (int i = n; i < numNodes; i++) {
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
            for (int i = 0; i < numNodes; i++) {
                for (int j = 0; j < numNodes; j++) {
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
