#include <TeamPlannerParam.hpp>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <iostream>

TeamPlannerParam::TeamPlannerParam() {
    flagOptimizeCost_ = true;
    flagTaskComplete_ = true;
    flagSprAddCutToSameType_ = true;
    flagFlowCover_ = true;
    flagConsiderSampleNum_ = false;
    taskCompleteReward_ = 100.0;
    timePenalty_ = 1.0;
    timePenalty1_ = 0.0;
    recoursePenalty_ = 1.0;
    taskRiskPenalty_ = 100.0;
    LARGETIME_ = 1e4;
    MAXTIME_ = 1e3;
    MAXENG_ = 1e12;

    flagSolver_ = TEAMPLANNER_SPR;

    CcpBeta_ = 0.95;
    taskBeta_ = 0.95;

    solverMaxTime_ = 200.0;
    solverIterMaxTime_ = 50.0;

    flagNotUseUnralavant_ = true;
    MAXALPHA_ = 20;

    taskNum_ = 7;
    vehNum_ = 18;
    capNum_ = 8;
    vehTypeNum_ = vehNum_;
    sampleNum_ = 0;
    randomType_ = TEAMPLANNER_GAUSSIAN;

    verboseLevel_ = 1;

    // std::vector<VehicleParam> vehParam_;
    // std::vector<TaskParam> taskParam_;

    // std::vector<double> vehEngCost_;
    // std::vector<bool> flagVehDistCost_;
    // task_cap_type = cell(n_task / n_task_rep, n_cap);
    // task_cap_num = cell(n_task / n_task_rep, n_cap);


}

TeamPlannerParam::~TeamPlannerParam() {
    clear();
}

void TeamPlannerParam::clear() {
    vehParam_.clear();
    taskParam_.clear();
    capType_.clear();
    vehNumPerType_.clear();
}

bool TeamPlannerParam::readFromFile(const std::string& fileName) {
    clear();
    try {
        YAML::Node yamlParam = YAML::LoadFile(fileName);
        if (yamlParam["flagOptimizeCost"]) {
            flagOptimizeCost_ = yamlParam["flagOptimizeCost"].as<bool>();
        }
        if (yamlParam["flagTaskComplete"]) {
            flagTaskComplete_ = yamlParam["flagTaskComplete"].as<bool>();
        }
        if (yamlParam["flagSprAddCutToSameType"]) {
            flagSprAddCutToSameType_ = yamlParam["flagSprAddCutToSameType"].as<bool>();
        }
        if (yamlParam["flagFlowCover"]) {
            flagFlowCover_ = yamlParam["flagFlowCover"].as<bool>();
        }
        if (yamlParam["flagConsiderSampleNum"]) {
            flagConsiderSampleNum_ = yamlParam["flagConsiderSampleNum"].as<bool>();
        }
        if (yamlParam["taskCompleteReward"]) {
            taskCompleteReward_ = yamlParam["taskCompleteReward"].as<double>();
        }
        if (yamlParam["timePenalty"]) {
            timePenalty_ = yamlParam["timePenalty"].as<double>();
        }
        if (yamlParam["recoursePenalty"]) {
            recoursePenalty_ = yamlParam["recoursePenalty"].as<double>();
        }
        if (yamlParam["taskRiskPenalty"]) {
            taskRiskPenalty_ = yamlParam["taskRiskPenalty"].as<double>();
        }
        if (yamlParam["LARGETIME"]) {
            LARGETIME_ = yamlParam["LARGETIME"].as<double>();
        }
        if (yamlParam["MAXTIME"]) {
            MAXTIME_ = yamlParam["MAXTIME"].as<double>();
        }
        if (yamlParam["MAXENG"]) {
            MAXENG_ = yamlParam["MAXENG"].as<double>();
        }
        if (yamlParam["flagSolver"]) {
            std::string flagSolverString = yamlParam["flagSolver"].as<std::string>();
            if (!flagSolverString.compare("TEAMPLANNER_SPR")) {
                flagSolver_ = TEAMPLANNER_SPR;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_CCP")) {
                flagSolver_ = TEAMPLANNER_CCP;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_SPRITER")) {
                flagSolver_ = TEAMPLANNER_SPRITER;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_CCPCONE")) {
                flagSolver_ = TEAMPLANNER_CCPCONE;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_CCPITER")) {
                flagSolver_ = TEAMPLANNER_CCPITER;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_TASKLIN")) {
                flagSolver_ = TEAMPLANNER_TASKLIN;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_TASKLSHAPED")) {
                flagSolver_ = TEAMPLANNER_TASKLSHAPED;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_TASKNON")) {
                flagSolver_ = TEAMPLANNER_TASKNON;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_CONDET")) {
                flagSolver_ = TEAMPLANNER_CONDET;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_CONTASKLIN")) {
                flagSolver_ = TEAMPLANNER_CONTASKLIN;
            }
            else if (!flagSolverString.compare("TEAMPLANNER_CONTASKLSHAPED")) {
                flagSolver_ = TEAMPLANNER_CONTASKLSHAPED;
            }
            else {
                flagSolver_ = TEAMPLANNER_DET;
            }
        }
        if (yamlParam["CcpBeta"]) {
            CcpBeta_ = yamlParam["CcpBeta"].as<double>();
        }
        if (yamlParam["taskBeta"]) {
            taskBeta_ = yamlParam["taskBeta"].as<double>();
        }
        if (yamlParam["solverMaxTime"]) {
            solverMaxTime_ = yamlParam["solverMaxTime"].as<double>();
        }
        if (yamlParam["solverIterMaxTime"]) {
            solverIterMaxTime_ = yamlParam["solverIterMaxTime"].as<double>();
        }
        if (yamlParam["flagNotUseUnralavant"]) {
            flagNotUseUnralavant_ = yamlParam["flagNotUseUnralavant"].as<bool>();
        }
        if (yamlParam["MAXALPHA"]) {
            MAXALPHA_ = yamlParam["MAXALPHA"].as<double>();
        }
        if (yamlParam["taskNum"]) {
            taskNum_ = yamlParam["taskNum"].as<int>();
        }
        if (yamlParam["vehNum"]) {
            vehNum_ = yamlParam["vehNum"].as<int>();
        }
        if (yamlParam["capNum"]) {
            capNum_ = yamlParam["capNum"].as<int>();
        }
        if (yamlParam["vehTypeNum"]) {
            vehTypeNum_ = yamlParam["vehTypeNum"].as<int>();
        }
        if (yamlParam["sampleNum"]) {
            sampleNum_ = yamlParam["sampleNum"].as<int>();
        }
        if (yamlParam["randomType"]) {
            int tempRandomType = yamlParam["randomType"].as<int>();
            randomType_ = static_cast<RandomType>(tempRandomType);
        }
        if (yamlParam["verboseLevel"]) {
            verboseLevel_ = yamlParam["verboseLevel"].as<int>();
        }
        if (yamlParam["capType"]) {
            capType_.resize(capNum_);
            for (int capId = 0; capId < capNum_; capId++) {
            capType_[capId] = yamlParam["capType"][capId].as<int>();
            }
        }
        if (yamlParam["vehNumPerType"]) {
            vehNumPerType_.resize(vehTypeNum_);
            for (int vehTypeId = 0; vehTypeId < vehTypeNum_; vehTypeId++) {
            vehNumPerType_[vehTypeId] = yamlParam["vehNumPerType"][vehTypeId].as<int>();
            }
        }
        if (yamlParam["vehicleParamFile"]) {
            std::string vehicleParamFile = yamlParam["vehicleParamFile"].as<std::string>();
            readVehicleFromFile(vehicleParamFile);
        }
        if (yamlParam["taskParamFile"]) {
            std::string taskParamFile = yamlParam["taskParamFile"].as<std::string>();
            readTaskFromFile(taskParamFile);
        }
        if (yamlParam["graphFile"]) {
            graphFile_ = yamlParam["graphFile"].as<std::string>();
        }
        if (yamlParam["sampleFile"]) {
            sampleFile_ = yamlParam["sampleFile"].as<std::string>();
        }
    }
    catch (const std::runtime_error &e) {
        std::cerr << e.what() << "\n";
        exit(-1);
        return false;
    }
    return true;
}

bool TeamPlannerParam::readTaskFromFile(const std::string& fileName) {
    try {
        YAML::Node yamlParam = YAML::LoadFile(fileName);
        for (int taskId = 0; ; taskId++) {
            std::string taskString = "task" + std::to_string(taskId);
            if (!yamlParam[taskString]) {
                break;
            }
            TaskParam aTaskParam;
            for (int andId = 0; ; andId++) {
                std::string andString = "and" + std::to_string(andId);
                if (!yamlParam[taskString][andString]) {
                    break;
                }
                std::vector<TaskReqGeq> aAnd;
                for (int orId = 0; ; orId++) {
                    std::string orString = "or" + std::to_string(orId);
                    if (!yamlParam[taskString][andString][orString]) {
                        break;
                    }
                    TaskReqGeq aOr;
                    aOr.capId_ = yamlParam[taskString][andString][orString]["capId"].as<int>();
                    aOr.capReq_ = yamlParam[taskString][andString][orString]["capReq"].as<double>();
                    if (yamlParam[taskString][andString][orString]["geq"]) {
                        aOr.geq_ = yamlParam[taskString][andString][orString]["geq"].as<bool>();
                    }
                    else {
                        aOr.geq_ = true;
                    }
                    if (yamlParam[taskString][andString][orString]["capVar"]) {
                        aOr.capVar_ = yamlParam[taskString][andString][orString]["capVar"].as<double>();
                    }
                    else {
                        aOr.capVar_ = 0.0;
                    }
                    aAnd.push_back(aOr);
                }
                aTaskParam.reqFcn_.push_back(aAnd);
            }
            taskParam_.push_back(aTaskParam);
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        exit(-1);
        return false;
    }
    return true;
}

bool TeamPlannerParam::readVehicleFromFile(const std::string& fileName) {
    try {
        YAML::Node yamlParam = YAML::LoadFile(fileName);
        for (int vId = 0; ; vId++) {
            std::string vString = "vehicle" + std::to_string(vId);
            if (!yamlParam[vString]) {
                break;
            }
            VehicleParam aVehParam;
            aVehParam.engCap_ = yamlParam[vString]["engCap"].as<double>();
            aVehParam.engCost_ = yamlParam[vString]["engCost"].as<double>();
            aVehParam.capSampleNum_.resize(capNum_);
            for (int capId = 0; capId < capNum_; capId++) {
                aVehParam.capVector_.push_back( yamlParam[vString]["capVector"][capId].as<double>() );
                if (yamlParam[vString]["capVar"]) {
                    aVehParam.capVar_.push_back( yamlParam[vString]["capVar"][capId].as<double>() );
                }
                if (yamlParam[vString]["capSampleNum"]) {
                    aVehParam.capSampleNum_[capId] = yamlParam[vString]["capSampleNum"][capId].as<int>();
                }
                else {
                    aVehParam.capSampleNum_[capId] = 0;
                }
            }
            vehParam_.push_back(aVehParam);
        }
        std::string rescueString = "rescueVehicle";
        if (yamlParam[rescueString]) {
            resqueVehParam_.engCap_ = yamlParam[rescueString]["engCap"].as<double>();
            resqueVehParam_.engCost_ = yamlParam[rescueString]["engCost"].as<double>();
            resqueVehParam_.capSampleNum_.resize(capNum_);
            for (int capId = 0; capId < capNum_; capId++) {
                resqueVehParam_.capVector_.push_back( yamlParam[rescueString]["capVector"][capId].as<double>() );
                if (yamlParam[rescueString]["capVar"]) {
                    resqueVehParam_.capVar_.push_back( yamlParam[rescueString]["capVar"][capId].as<double>() );
                }
                if (yamlParam[rescueString]["capSampleNum"]) {
                    resqueVehParam_.capSampleNum_[capId] = yamlParam[rescueString]["capSampleNum"][capId].as<double>();
                }
                else {
                    resqueVehParam_.capSampleNum_[capId] = 0;
                }
            }
        }
        else {
            resqueVehParam_ = vehParam_[0];
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        exit(-1);
        return false;
    }
    return true;
}

void TeamPlannerParam::print() const {
    std::cout << "Team planner param:\n";
    std::cout << "flagOptimizeCost = " << flagOptimizeCost_ << "\n";
    std::cout << "flagTaskComplete = " << flagTaskComplete_ << "\n";
    std::cout << "flagFlowCover = " << flagFlowCover_ << "\n";
    std::cout << "flagConsiderSampleNum = " << flagConsiderSampleNum_ << "\n";
    std::cout << "taskCompleteReward = " << taskCompleteReward_ << "\n";
    std::cout << "timePenalty = " << timePenalty_ << "\n";
    std::cout << "recoursePenalty = " << recoursePenalty_ << "\n";
    std::cout << "taskRiskPenalty = " << taskRiskPenalty_ << "\n";
    std::cout << "LARGETIME = " << LARGETIME_ << "\n";
    std::cout << "MAXTIME = " << MAXTIME_ << "\n";
    std::cout << "MAXENG = " << MAXENG_ << "\n";
    std::cout << "flagSolver = " << flagSolver_ << "\n";
    std::cout << "CcpBeta = " << CcpBeta_ << "\n";
    std::cout << "taskBeta = " << taskBeta_ << "\n";
    std::cout << "solverMaxTime = " << solverMaxTime_ << "\n";
    std::cout << "solverIterMaxTime = " << solverIterMaxTime_ << "\n";
    std::cout << "flagNotUseUnralavant = " << flagNotUseUnralavant_ << "\n";
    std::cout << "MAXALPHA = " << MAXALPHA_ << "\n";
    std::cout << "graphFile = " << graphFile_ << "\n";
    std::cout << "sampleFile = " << sampleFile_ << "\n";
    std::cout << "taskNum = " << taskNum_ << "\n";
    std::cout << "vehNum = " << vehNum_ << "\n";
    std::cout << "capNum = " << capNum_ << "\n";
    std::cout << "verboseLevel = " << verboseLevel_ << "\n";
    int offset = 1;
    if (sampleNum_ > 0) {
        std::cout << "sampleNum = " << sampleNum_ << "\n";
        std::cout << "randomType = " << randomType_ << "\n";
    }
    std::cout << "capType: [";
    for (int capId = 0; capId < capNum_; capId++) {
        if (capId > 0) {
            std::cout << ",\t";
        }
        std::cout << capType_[capId];
    }
    std::cout << "]\n";
    std::cout << "vehNumPerType: [";
    for (int vehTypeId = 0; vehTypeId < vehTypeNum_; vehTypeId++) {
        if (vehTypeId > 0) {
            std::cout << ",\t";
        }
        std::cout << vehNumPerType_[vehTypeId];
    }
    std::cout << "]\n";
    std::cout << "Vehicle:\n";
    for (int vId = 0; vId < static_cast<int>(vehParam_.size()); vId++) {
        std::cout << vId+offset << ":\t" << vehParam_[vId].toString() << "\n";
    }
    std::cout << "rescue vehicle:\t" << resqueVehParam_.toString() << "\n";
    std::cout << "Task:\n";
    for (int taskId = 0; taskId < static_cast<int>(taskParam_.size()); taskId++) {
        std::cout << taskId+offset << ":\t" << taskParam_[taskId].toString() << "\n";
    }
    std::cout << "\n";
}
