#pragma once

#include <ilcplex/ilocplex.h>

class CplexManager {
public:
    static CplexManager& getInstance() {
        static CplexManager instance;
        return instance;
    }

    ~CplexManager() {
        env.end();
    }

    IloEnv& getEnv() { return env; }

    CplexManager(const CplexManager&) = delete;
    CplexManager& operator=(const CplexManager&) = delete;

private:
    CplexManager() {}

    IloEnv env;
};
