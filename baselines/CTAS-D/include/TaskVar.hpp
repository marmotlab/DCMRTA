#ifndef TEAMPLANNER_TASKVAR_HPP
#define TEAMPLANNER_TASKVAR_HPP

#include <vector>
// #include <string>
#include <random>
#include <TaskParam.hpp>
#include <gurobi_c++.h>
#include <ConstantDef.hpp>

class TaskVarGeq
{
public:
    int capId_;
    std::vector<double> capSample_;
    GRBVar riskVar_;
    GRBVar helpVar_;
    GRBVar lbVar_;
    GRBVar* sampleVar_;
    double probSuccess_;
    double riskCost_;
    double helpValue_;
    double lbValue_;
public:
    TaskVarGeq();
    ~TaskVarGeq();
    void clear();
    // std::string toString() const;
};

class TaskVar
{
public:
    double probSuccess_;
    double riskCost_;
    std::vector<std::vector<TaskVarGeq>> reqFcn_;
public:
    TaskVar();
    ~TaskVar();
    void clear();
    void genSample(const TaskParam& taskParam, std::default_random_engine& generator, int sampleNum, RandomType flagRandom);
    std::string toString() const;
};

#endif //TEAMPLANNER_TASKVAR_HPP