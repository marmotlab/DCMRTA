#ifndef TEAMPLANNER_TEAMPLANNERCALLBACK_HPP
#define TEAMPLANNER_TEAMPLANNERCALLBACK_HPP

#include <fstream>
#include <gurobi_c++.h>
#include <TeamPlanner.hpp>

class TeamPlannerCallback: public GRBCallback
{
  public:
    double lastIter_;
    double lastNode_;
    int varNum_;
    GRBVar* var_;
    TeamPlanner* planner_;
    std::ofstream* logFile_;
    TeamPlannerCallback();
    TeamPlannerCallback(int varNum, GRBVar* var, TeamPlanner* planner, std::ofstream* logFile = nullptr);
    ~TeamPlannerCallback();
  protected:
    void callback();
};



#endif // TEAMPLANNER_TEAMPLANNERCALLBACK_HPP