#include <TeamPlanner.hpp>

void TeamPlanner::initializeModel() {
    env_ = new GRBEnv();
    if (param_.verboseLevel_ >= 1) {
        env_->set(GRB_IntParam_OutputFlag, 1);
    }
    else {
        env_->set(GRB_IntParam_OutputFlag, 0);
    }

    model_ = new GRBModel(*env_);

    if (param_.flagSolver_ == TEAMPLANNER_CONDET || param_.flagSolver_ == TEAMPLANNER_CONTASKLIN || param_.flagSolver_ == TEAMPLANNER_CONTASKLSHAPED) {
        flagContinuous_ = true;
    }

    if (param_.solverMaxTime_ > 0.001) {
        model_->set(GRB_DoubleParam_TimeLimit, param_.solverMaxTime_);
    }

    if (flagContinuous_) {
        xVar_ = model_->addVars(xVarNum_, GRB_CONTINUOUS);
        yVar_ = model_->addVars(yVarNum_, GRB_CONTINUOUS);
        xrVar_ = model_->addVars(xVarNum_, GRB_BINARY);
        yrVar_ = model_->addVars(yVarNum_, GRB_BINARY);
        gVar_ = model_->addVars(gVarNum_, GRB_CONTINUOUS);
    }
    else {
        xVar_ = model_->addVars(xVarNum_, GRB_BINARY);
        yVar_ = model_->addVars(yVarNum_, GRB_BINARY);
    }
    zVar_ = model_->addVars(zVarNum_, GRB_BINARY);
    qVar_ = model_->addVars(qVarNum_, GRB_CONTINUOUS);
    if (param_.flagSolver_ == TEAMPLANNER_SPR || param_.flagSolver_ == TEAMPLANNER_SPRITER) {
        thetaVar_ = model_->addVars(vehNum_, GRB_CONTINUOUS);
    }

    sumCap_.resize(capNum_);
    for (int c = 0; c < capNum_; c++) {
        sumCap_[c] = 0.0;
        for (int k = 0; k < vehNum_; k++) {
            if (flagContinuous_) {
                sumCap_[c] += param_.vehParam_[k].capVector_[c] * static_cast<double>(param_.vehNumPerType_[k]);
            }
            else {
                sumCap_[c] += param_.vehParam_[k].capVector_[c];
            }
        }
    }

    if (param_.flagSolver_ == TEAMPLANNER_TASKLIN    || param_.flagSolver_ == TEAMPLANNER_TASKLSHAPED     || param_.flagSolver_ == TEAMPLANNER_TASKNON
     || param_.flagSolver_ == TEAMPLANNER_CONTASKLIN || param_.flagSolver_ == TEAMPLANNER_CONTASKLSHAPED) {
        flagTaskVarAdded_ = true;
    }

    vehVar_.resize(vehNum_);
    taskVar_.resize(taskNum_);
    genSample();
    if (flagTaskVarAdded_ && param_.randomType_ == RandomType::TEAMPLANNER_READ) {
        readSamples(param_.sampleFile_);
    }
}

bool TeamPlanner::initializeNum() {
    vehTypeNum_ = param_.vehTypeNum_;
    vehNum_ = param_.vehNum_;
    taskNum_ = param_.taskNum_;
    capNum_ = param_.capNum_;

    edgeNum_ = static_cast<int>(graph_[0].edgeNum());
    nodeNum_ = static_cast<int>(graph_[0].nodeNum());
    edgeOffset_.clear();
    for (int iVeh = 0; iVeh<vehNum_; iVeh++) {
        edgeOffset_.push_back(iVeh*edgeNum_);
    }

    xVarNum_ = vehNum_ * edgeNum_;
    yVarNum_ = vehNum_ * taskNum_;
    zVarNum_ = taskNum_;
    qVarNum_ = nodeNum_;
    gVarNum_ = vehNum_ * (taskNum_ + 2);

    return true;
}

void TeamPlanner::genSample() {
    if (!flagTaskVarAdded_) {
        for (int k = 0; k < vehNum_; k++) {
            vehVar_[k].genSample(param_.vehParam_[k], generator_, 0, param_.randomType_);
        }
        for (int i = 0; i < taskNum_; i++) {
            taskVar_[i].genSample(param_.taskParam_[i], generator_, 0, param_.randomType_);
        }
        return;
    }
    for (int k = 0; k < vehNum_; k++) {
        if (param_.flagSolver_ == TEAMPLANNER_TASKNON) {
            vehVar_[k].genSample(param_.vehParam_[k], generator_, 0, param_.randomType_);
        }
        else {
            vehVar_[k].genSample(param_.vehParam_[k], generator_, param_.sampleNum_, param_.randomType_);
        }
    }
    for (int i = 0; i < taskNum_; i++) {
        if (param_.flagSolver_ == TEAMPLANNER_TASKNON) {
            taskVar_[i].genSample(param_.taskParam_[i], generator_, 0, param_.randomType_);
        }
        else {
            taskVar_[i].genSample(param_.taskParam_[i], generator_, param_.sampleNum_, param_.randomType_);
        }
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                // Initialize variables
                // Add risk variable (h)
                double varLB = -1e10;
                double varUB = 1e10;
                double varObj = param_.taskRiskPenalty_ / param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                std::string commonName  = toString(TEAMPLANNER_TASK, i)     + ","
                                        + toString(TEAMPLANNER_AND , andId) + ","
                                        + toString(TEAMPLANNER_OR  , orId);
                std::string varName = "risk[" + commonName + "]";
                taskVar_[i].reqFcn_[andId][orId].riskVar_ = model_->addVar(varLB, varUB, varObj, GRB_CONTINUOUS, varName);
                // Add helper variable (lambda)
                varLB = -1e10;
                varUB = 1e10;
                varObj = 0.0;
                varName = "helper[" + commonName + "]";
                taskVar_[i].reqFcn_[andId][orId].helpVar_ = model_->addVar(varLB, varUB, varObj, GRB_CONTINUOUS, varName);
                if (param_.flagSolver_ == TEAMPLANNER_TASKLIN || param_.flagSolver_ == TEAMPLANNER_CONTASKLIN) {
                    // Add sample variable (w_{m1 a3}^{\xi})
                    taskVar_[i].reqFcn_[andId][orId].sampleVar_ = model_->addVars(param_.sampleNum_, GRB_CONTINUOUS);
                    for (int sampleId = 0; sampleId < param_.sampleNum_; sampleId++) {
                        varLB = 0.0;
                        varUB = 1e10;
                        varObj = 0.0;
                        varName = "sample[" + commonName + std::to_string(sampleId) + "]";
                        taskVar_[i].reqFcn_[andId][orId].sampleVar_[sampleId].set(GRB_DoubleAttr_Obj, varObj);
                        taskVar_[i].reqFcn_[andId][orId].sampleVar_[sampleId].set(GRB_StringAttr_VarName, varName);
                        taskVar_[i].reqFcn_[andId][orId].sampleVar_[sampleId].set(GRB_DoubleAttr_LB, varLB);
                        taskVar_[i].reqFcn_[andId][orId].sampleVar_[sampleId].set(GRB_DoubleAttr_UB, varUB);
                    }
                }
                if (param_.flagSolver_ == TEAMPLANNER_TASKLSHAPED || param_.flagSolver_ == TEAMPLANNER_TASKNON || param_.flagSolver_ == TEAMPLANNER_CONTASKLSHAPED) {
                    // Add lower bounding variable (theta_{m1 a3})
                    varLB = 0.0;
                    varUB = 1e10;
                    varObj = 0.0;
                    varName = "lbVar[" + commonName + "]";
                    taskVar_[i].reqFcn_[andId][orId].lbVar_ = model_->addVar(varLB, varUB, varObj, GRB_CONTINUOUS, varName);
                }
            }
        }
    }
}

bool TeamPlanner::readSamples(const std::string& fileName) {
    try {
        YAML::Node yamlParam = YAML::LoadFile(fileName);
        for (int i = 0; i < taskNum_; i++) {
            std::string taskString = "task" + std::to_string(i);
            for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
                std::string andString = "and" + std::to_string(andId);
                for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                    std::string orString = "or" + std::to_string(orId);
                    TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                    for (size_t s = 0; s < aGeq.capSample_.size(); s++) {
                        aGeq.capSample_[s] = yamlParam[taskString][andString][orString][s].as<double>();
                    }
                }
            }
        }
        for (int k = 0; k < vehNum_; k++) {
            std::string vString = "vehicle" + std::to_string(k);
            for (size_t a = 0; a < vehVar_[k].capSample_.size(); a++) {
                std::string capString = "cap" + std::to_string(a);
                for (size_t s = 0; s < vehVar_[k].capSample_[a].size(); s++) {
                    vehVar_[k].capSample_[a][s] = yamlParam[vString][capString][s].as<double>();
                }
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        exit(-1);
        return false;
    }
    return true;
}

bool TeamPlanner::getParams(const std::string& fileName) {
    param_.readFromFile(fileName);
    if (param_.verboseLevel_ >= 1) {
        param_.print();
    }
    return true;
}

bool TeamPlanner::getGraph(int vehNum, int nodeNum, const std::string& fileName) {
    try {
        YAML::Node yamlParam = YAML::LoadFile(fileName);
        graph_.clear();
        graph_.resize(vehNum);
        for (int vId = 0; vId < vehNum; vId++) {
            std::string vString = "vehicle" + std::to_string(vId);
            if (!yamlParam[vString]) {
                break;
            }
            for (int nodeId = 0; nodeId < nodeNum; nodeId++) {
                 std::string nodeString = "node" + std::to_string(nodeId);
                double nodeTimeCost;
                 if (!yamlParam[vString][nodeString]) {
                     nodeTimeCost = 0.0;
                 }
                 else {
                     nodeTimeCost = yamlParam[vString][nodeString].as<double>();
                 }
                graph_[vId].addNode(nodeId, nodeTimeCost);
            }
            for (int edgeId = 0; ; edgeId++) {
                std::string edgeString = "edge" + std::to_string(edgeId);
                if (!yamlParam[vString][edgeString]) {
                    break;
                }
                int node1       = yamlParam[vString][edgeString][0].as<int>();
                int node2       = yamlParam[vString][edgeString][1].as<int>();
                int type        = yamlParam[vString][edgeString][2].as<int>();
                double engCost  = yamlParam[vString][edgeString][3].as<double>();
                double engUnc   = yamlParam[vString][edgeString][4].as<double>();
                double timeCost = yamlParam[vString][edgeString][5].as<double>();
                bool flagDirected = true;
                graph_[vId].addEdge(node1, node2, type, engCost, engUnc, timeCost, flagDirected);
            }
        }
        // for (int vId = 0; vId < vehNum; vId++) {
        //     std::cout << "vehicle" << vId << "\n";
        //     graph_[vId].print();
        // }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        exit(-1);
        return false;
    }
    return true;
}
