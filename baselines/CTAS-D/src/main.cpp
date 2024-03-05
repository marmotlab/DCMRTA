#include <iostream>
#include <string>
#include <TeamPlanner.hpp>

int main(int argc, char const *argv[])
{
    std::string paramFile;
    std::string logFile;
    std::string sampleFile;
    if (argc < 2) {
        paramFile = "../config/env_0/planner_param.yaml";
    }
    else {
        paramFile = argv[1];
    }
    if (argc < 3) {
        logFile = "testLog.yaml";
    }
    else {
        logFile = argv[2];
    }
    if (argc < 4) {
        sampleFile = "sampleLog.yaml";
    }
    else {
        sampleFile = argv[3];
    }
    TeamPlanner planner;
    planner.formProblem(paramFile);
    planner.saveSamples(sampleFile);
    planner.optimize();
    planner.printSolution();
    planner.saveSolution(logFile, paramFile);

    std::cout << "Haha!\n";
    return 0;
}
