#ifndef TEAMPLANNER_TEAMPLANNER_HPP
#define TEAMPLANNER_TEAMPLANNER_HPP

#include <chrono>
#include <string>
#include <random>
#include <gurobi_c++.h>
#include <TeamPlannerParam.hpp>
#include <Graph.hpp>
#include <VehicleVar.hpp>
#include <TaskVar.hpp>

enum SymbolType{
    TEAMPLANNER_VEHC = 0,
    TEAMPLANNER_NODE = 1,
    TEAMPLANNER_TASK = 2,
    TEAMPLANNER_PATH = 3,
    TEAMPLANNER_AND = 4,
    TEAMPLANNER_OR = 5,
    TEAMPLANNER_VEHCTYPE = 6
};

class TeamPlannerSolution
{
public:
    double totalCost_;
    double* xVar_;
    double* yVar_;
    double* zVar_;
    double* qVar_;
    double* thetaVar_;
    double* alphaVar_;
    double* wVar_;

public:
    TeamPlannerSolution();
    ~TeamPlannerSolution();
    void clear();
    void initialize(int xVarNum, int yVarNum, int zVarNum, int qVarNum, int thetaVarNum, int alphaVarNum, int wVarNum);
};

class TeamPlanner
{
protected:
    GRBEnv* env_;
    GRBModel* model_;
    GRBVar* xVar_; // binary variable, == 1 if the corresponding edge is selected, otherwise 0
    GRBVar* yVar_; // binary variable, == 1 if the node is visited by vehicle, otherwise 0
    GRBVar* zVar_; // binary variable, == 1 if the task is completed, otherwise 0
    GRBVar* qVar_; // continuous variable, the time that the task starts
    GRBVar* thetaVar_; // continuous variable, linear relexation of the recourse cost of each vehicle
    GRBVar* xrVar_; // binary variable, == 1 if the corresponding edge is selected, otherwise 0
    GRBVar* yrVar_; // binary variable, == 1 if the node is visited by vehicle, otherwise 0
    GRBVar* gVar_; // continuous variable, the maximum energy spent by one of the vehicle at a node
    std::vector<GRBVar> alphaVar_;
    std::vector<GRBVar> wVar_;
    TeamPlannerSolution bestSolution_;
    
    TeamPlannerParam param_;
    std::vector<Graph> graph_;

    int vehTypeNum_;
    int vehNum_;
    int taskNum_;
    int capNum_;
    int edgeNum_;
    int nodeNum_;
    std::vector<int> edgeOffset_;
    std::vector<double> sumCap_;

    int xVarNum_; //    xVarNum_ = vehNum_ * edgeNum_; *
    int yVarNum_; //    yVarNum_ = vehNum_ * taskNum_;
    int zVarNum_; //    zVarNum_ = taskNum_;
    int qVarNum_; //    qVarNum_ = nodeNum_;
    int gVarNum_; //    gVarNum_ = vehNum_ * (taskNum_ + 2);

	std::chrono::system_clock::time_point startTime_;
	double prevTime_;
	double solverTime_;

    int constraintNum_;
    bool flagOptimized_;
    bool flagSuccess_;
    bool flagTaskVarAdded_;
    bool flagContinuous_;

    std::default_random_engine generator_;
    std::vector<TaskVar> taskVar_;
    std::vector<VehicleVar> vehVar_;

public:
    int cutNum_;
    // Basic - TeamPlannerBasic.cpp
    TeamPlanner();
    ~TeamPlanner();
    void clear();
    double GetTime() const;
    double GetTimeDuration();
    // Copy solution - TeamPlannerCopy.cpp
    void copyPreviousSolution(const std::vector<double>& vehRecourseCost);
    void initializeBestSolution();
    void copyToBestSolution(double totalCost);
    // Evaluation and print - TeamPlannerPrint.cpp
    std::string toString(SymbolType type, int id, bool flagOffset = false, bool flagNumberOnly = false) const;
    template<class T> std::string toString(const std::vector<T>& data) const;
    void getCost(const std::vector<double>& vehRecourseCost,
                double& energyCost, double& timeCost, double& otherCost,
                double& fCost, double& thetaCost, double& recourseCost,
                double& oriLinCost, double& oriNonCost,
                double& oriRiskLinCost, double& oriRiskNonCost);
    void getTeam(std::vector<std::vector<int>>& taskTeam, const double* yValue = nullptr) const;
    void getPath(std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue = nullptr) const;
    void getPathContinuous(std::vector<int>& vehType, std::vector<double>& vehFlow, std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue = nullptr) const;
    void getPathCover(std::vector<int>& vehType, std::vector<double>& vehFlow, std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue = nullptr) const;
    void getTeamContinuous(std::vector<std::vector<int>>& taskTeam, std::vector<std::vector<double>>& taskTeamDense, const std::vector<int>& vehType, const std::vector<double>& vehFlow, const std::vector<std::vector<int>>& vehPath) const;
    void getFinalCost(const std::vector<int>& vehType, const std::vector<double>& vehFlow, const std::vector<std::vector<int>>& vehPath,
                    const std::vector<std::vector<int>>& vehEdge, const std::vector<double>& vehSumEng, const std::vector<double>& vehSumVar,
                    const std::vector<std::vector<int>>& taskTeam, const std::vector<std::vector<double>>& taskTeamDense,
                    double& finalLinCost, double& finalNonCost, double& taskRiskLinCost, double& taskRiskNonCost);
    // void printSolution();
    void printSolution(std::vector<std::vector<int>>* vehPathOut = nullptr, std::vector<std::vector<int>>* taskTeamOut = nullptr, std::vector<std::vector<double>>* taskTeamDenseOut = nullptr);
    void saveSolution(const std::string& saveFileName, const std::string& paramFileName);
    bool readSamples(const std::string& fileName);
    void saveSamples(const std::string& saveFileName) const;
    // Initialize model - TeamPlannerInitialize.cpp
    void initializeModel();
    bool initializeNum();
    void genSample();
    bool getParams(const std::string& fileName);
    bool getGraph(int nodeNum, int vehNum, const std::string& fileName);
    // Main functions - TeamPlannerMain.cpp
    bool formProblem(const std::string& paramFile);
    bool optimize();
    // Common model - TeamPlannerCommonModel.cpp
    void formVarNameCost();
    void formCommonModel();
    void formDeterministicEnergy();
    // Stochastic energy model - TeamPlannerEngModel.cpp
    void formCCPEnergy();
    bool getRecourseCost(std::vector<double>& vehRecourseCost, const double* xValue = nullptr) const;
    bool addRecourseCut(double* xValue, double* thetaValue, std::vector<GRBLinExpr>& recourseCut, std::vector<double>& vehRecourseCost) const;
    bool addCCPCut(double* xValue, std::vector<GRBLinExpr>& ccpCut) const;
    // Stochastic task model - TeamPlannerTaskModel.cpp
    void formTaskRiskCost();
    void formTaskRiskCostLShaped();
    bool addTaskLShapedCut(double* yValue, std::vector<GRBLinExpr>& recourseCut) const;
    bool addTaskIntLShapedCut(double* yValue, std::vector<GRBLinExpr>& recourseCut) const;
    bool getProbSuccess(double& riskCostNon, double& riskCostLin, const std::vector<std::vector<double>>* taskTeamDense = nullptr);
    // Common continuous model - TeamPlannerContinuousModel.cpp
    void formContinuousModel();
    void formContinuousDetEnergy();

    inline int sub2xId(int veh, int node1, int node2, int type) const {
        int edgeId = graph_[veh].edge2id(node1, node2, type);
        if (edgeId < 0) {
            return -100;
        }
        else {
            return (edgeId + edgeOffset_[veh]);
        }
    }
    inline int sub2xId(int veh, int edgeId) const {
        return (edgeId + edgeOffset_[veh]);
    }
    inline bool xId2edge(int id, int& veh, GraphEdgeParam& edgeParam) const {
        // Assume the graph edgeNum_ are the same
        veh = id / edgeNum_;
        int edgeId = id - veh * edgeNum_;
        return graph_[veh].id2edge(edgeId, edgeParam);
    }
    inline int sub2yId(int veh, int task) const {
        return (task + veh * taskNum_);
    }
    inline bool yId2sub(int id, int& veh, int& task) const {
        veh = id / taskNum_;
        task = id - veh * taskNum_;
        return true;
    }
    inline int sub2gId(int veh, int node) const {
        int id = veh * (taskNum_+2);
        if (node < taskNum_) {
            id += node;
        }
        else if (node < taskNum_ + vehNum_) {
            id += taskNum_;
        }
        else {
            id += taskNum_ + 1;
        }
        return id;
    }
    inline bool gId2sub(int id, int& veh, int& node) const {
        veh = id / (taskNum_+2);
        node = id - veh * (taskNum_+2);
        if (node == taskNum_) {
            node = taskNum_ + veh;
        }
        else if (node == taskNum_ + 1) {
            node = taskNum_ + vehNum_ + veh;
        }
        return true;
    }
    inline int xVarNum() const {
        return xVarNum_;
    }
    inline int yVarNum() const {
        return yVarNum_;
    }
    inline int vehNum() const {
        return vehNum_;
    }
    inline int taskNum() const {
        return taskNum_;
    }
    inline const GRBVar* xVar() const {
        return xVar_;
    }
    inline const GRBVar* yVar() const {
        return yVar_;
    }
    inline const GRBVar* thetaVar() const {
        return thetaVar_;
    }
    inline std::vector<TaskVar>* taskVar() {
        return &taskVar_;
    }
    inline std::vector<VehicleVar>* vehVar() {
        return &vehVar_;
    }
    inline const TeamPlannerParam& param() const {
        return param_;
    }
    inline void update() {
        model_->update();
    }
    inline void setLazy() {
        std::cout << "before: " << model_->get(GRB_IntParam_LazyConstraints) << "\n";
        if (model_->get(GRB_IntParam_LazyConstraints) == 0) {
            model_->set(GRB_IntParam_LazyConstraints, 1);
        }
        std::cout << "after : " << model_->get(GRB_IntParam_LazyConstraints) << "\n";
    }
};


#endif //TEAMPLANNER_TEAMPLANNER_HPP