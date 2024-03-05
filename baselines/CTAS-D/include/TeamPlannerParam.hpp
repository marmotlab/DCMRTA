#ifndef TEAMPLANNER_TEAMPLANNERPARAM_HPP
#define TEAMPLANNER_TEAMPLANNERPARAM_HPP

#include <vector>
#include <string>
#include <VehicleParam.hpp>
#include <TaskParam.hpp>
#include <ConstantDef.hpp>

class TeamPlannerParam
{
public:
    bool flagOptimizeCost_;
    bool flagTaskComplete_;
    bool flagSprAddCutToSameType_;
    bool flagFlowCover_;
    bool flagConsiderSampleNum_;
    double taskCompleteReward_;
    double timePenalty_;
    double timePenalty1_;
    double recoursePenalty_;
    double taskRiskPenalty_;
    double LARGETIME_;
    double MAXTIME_;
    double MAXENG_;

    ModelType flagSolver_;

    double CcpBeta_;
    double taskBeta_;
    double solverMaxTime_;
    double solverIterMaxTime_;

    bool flagNotUseUnralavant_;
    double MAXALPHA_;
    std::string graphFile_;
    std::string sampleFile_;

    int taskNum_;
    int vehNum_;
    int capNum_;
    int vehTypeNum_;
    int sampleNum_;
    RandomType randomType_;
    int verboseLevel_;

    std::vector<VehicleParam> vehParam_;
    std::vector<TaskParam> taskParam_;
    VehicleParam resqueVehParam_;
    std::vector<int> capType_;
    std::vector<int> vehNumPerType_;

public:
    TeamPlannerParam();
    ~TeamPlannerParam();
    void clear();
    bool readFromFile(const std::string& fileName);
    bool readVehicleFromFile(const std::string& fileName);
    bool readTaskFromFile(const std::string& fileName);
    void print() const;
};

#endif //TEAMPLANNER_TEAMPLANNERPARAM_HPP