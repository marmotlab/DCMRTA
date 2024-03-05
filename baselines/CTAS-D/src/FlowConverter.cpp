#include <FlowConverter.hpp>
#include <LocalMath.hpp>
#include <math.h>       /* ceil */
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <iostream>
#include <fstream>

FlowConverter::FlowConverter() {
    env_ = nullptr;
    roundModel_ = nullptr;
    coverModel_ = nullptr;
    intFlowVar_ = nullptr;
    coverFlowVar_ = nullptr;
    edgeNum_ = -1;
    nodeNum_ = -1;
    startNode_ = -1;
    endNode_ = -1;
    intFlow_ = -1;
    oriFlow_ = -1.0;
    roundOptimized_ = false;
    coverOptimized_ = false;
    verboseLevel_ = 3;
}

FlowConverter::~FlowConverter() {
}

void FlowConverter::clear() {
    if (intFlowVar_ != nullptr) {
        delete[] intFlowVar_;
        intFlowVar_ = nullptr;
    }
    if (coverFlowVar_ != nullptr) {
        delete[] coverFlowVar_;
        coverFlowVar_ = nullptr;
    }
    if (roundModel_ != nullptr) {
        delete roundModel_;
        roundModel_ = nullptr;
    }
    if (coverModel_ != nullptr) {
        delete coverModel_;
        coverModel_ = nullptr;
    }
    if (env_ != nullptr) {
        delete env_;
        env_ = nullptr;
    }
    graph_.clear();
    edgeNum_ = -1;
    nodeNum_ = -1;
    startNode_ = -1;
    endNode_ = -1;
    intFlow_ = -1;
    oriFlow_ = -1.0;
    roundOptimized_ = false;
    coverOptimized_ = false;
}

void FlowConverter::initializeEnv() {
    clear();
    env_ = new GRBEnv();
    if (verboseLevel_ >= 3) {
        env_->set(GRB_IntParam_OutputFlag, 1);
    }
    else {
        env_->set(GRB_IntParam_OutputFlag, 0);
    }
}

bool FlowConverter::formRoundProblem(const std::string& paramFile) {
    graph_.readFromFile(paramFile, "vehicle0");
    graph_.print();
    YAML::Node yamlParam = YAML::LoadFile(paramFile);
    startNode_ = yamlParam["vehicle0"]["startNode"].as<int>();
    endNode_   = yamlParam["vehicle0"]["endNode"  ].as<int>();
    return formRoundProblem();
}

bool FlowConverter::formRoundProblem(int startNode, int endNode) {
    startNode_ = startNode;
    endNode_ = endNode;
    return formRoundProblem();
}

bool FlowConverter::formRoundProblem() {
    edgeNum_ = graph_.edgeNum();
    nodeNum_ = graph_.nodeNum();
    initializeRoundModel();
    formRoundConstraints(startNode_, endNode_);
    return true;
}

bool FlowConverter::formCoverProblem() {
    if (!roundOptimized_) {
        return false;
    }
    initializeCoverModel();
    formCoverConstraints();
    return true;
}

void FlowConverter::initializeRoundModel() {
    double myEps = 0.0001;
    // double myEps = 0.0;
    roundModel_ = new GRBModel(*env_);
    intFlowVar_ = roundModel_->addVars(graph_.edgeNum(), GRB_CONTINUOUS);
    for (int edgeId = 0; edgeId < edgeNum_; edgeId++) {
        const GraphEdgeParam& edgeParam = graph_.edge(edgeId);
        double flowLB = ceil(edgeParam.timeCost_ - myEps); // TODO HARD CODE
        double flowUB = edgeParam.engUnc_; // TODO HARD CODE
        // double flowLB = 1.0; // TODO HARD CODE
        // double flowUB = 100.0; // TODO HARD CODE
        std::string varName = "f["  + std::to_string(edgeParam.edge_.node1_) + ","
                                    + std::to_string(edgeParam.edge_.node2_) + ","
                                    + std::to_string(edgeParam.edge_.type_)  + "]";

        intFlowVar_[edgeId].set(GRB_DoubleAttr_Obj, edgeParam.engCost_);
        intFlowVar_[edgeId].set(GRB_StringAttr_VarName, varName);
        intFlowVar_[edgeId].set(GRB_DoubleAttr_LB, flowLB);
        intFlowVar_[edgeId].set(GRB_DoubleAttr_UB, flowUB);
    }

    int startId = graph_.node2id(startNode_);
    oriFlow_ = 0.0;
    for (int edgeId : graph_.node(startId).outEdgeIds_) { // for ( auto it = myset.begin(); it != myset.end(); ++it )
        const GraphEdgeParam& edgeParam = graph_.edge(edgeId);
        oriFlow_    += edgeParam.timeCost_;
    }
}

void FlowConverter::initializeCoverModel() {
    coverModel_ = new GRBModel(*env_);
    // Initialize flow variables
    coverFlowVar_ = coverModel_->addVars(intFlow_ * edgeNum_, GRB_BINARY);
    for (int k = 0; k < intFlow_; k++) {
        for (int edgeId = 0; edgeId < edgeNum_; edgeId++) {
            // const GraphEdgeParam& edgeParam = graph_.edge(edgeId);
            int coverId = cover2id(k, edgeId);
            coverFlowVar_[edgeId].set(GRB_DoubleAttr_Obj, 0.0);
            // intFlowVar_[edgeId].set(GRB_StringAttr_VarName, varName);
            // intFlowVar_[edgeId].set(GRB_DoubleAttr_LB, flowLB);
            // intFlowVar_[edgeId].set(GRB_DoubleAttr_UB, flowUB);
        }
    }
    // Initialize vehicle energy cost variables
    double varLB = 0.0;
    double varUB = 1e10;
    double varObj = 1.0;
    std::string varName = "maxEngVar";
    vehVar_ = coverModel_->addVar(varLB, varUB, varObj, GRB_CONTINUOUS, varName);
}

void FlowConverter::formRoundConstraints(int startNode, int endNode) {
    // Flow Constraint: incoming edges == outgoing edges
    // const int nodeNum = graph_.nodeNum();
    for (int i = 0; i < nodeNum_; i++) {
        // Skip the start (source) and end node (sink)
        if (graph_.node(i).node_ == startNode || graph_.node(i).node_ == endNode) {
            continue;
        }
        GRBLinExpr constraint = 0;
        for (int edgeId : graph_.node(i).inEdgeIds_) { // for ( auto it = myset.begin(); it != myset.end(); ++it )
            constraint += intFlowVar_[edgeId];
        }
        for (int edgeId : graph_.node(i).outEdgeIds_) {
            constraint -= intFlowVar_[edgeId];
        }
        std::string constraintName = "FlowInOut[" + std::to_string(i) + "]";
        roundModel_->addConstr(constraint == 0, constraintName);
    }
}

void FlowConverter::formCoverConstraints() {
    // Flow Constraint: incoming edges == outgoing edges
    for (int i = 0; i < nodeNum_; i++) {
        // Skip the start (source) and end node (sink)
        if (graph_.node(i).node_ == startNode_ || graph_.node(i).node_ == endNode_) {
            continue;
        }
        for (int k = 0; k < intFlow_; k++) {
            GRBLinExpr constraint = 0;
            for (int edgeId : graph_.node(i).inEdgeIds_) { // for ( auto it = myset.begin(); it != myset.end(); ++it )
                int coverId = cover2id(k, edgeId);
                constraint += coverFlowVar_[coverId];
            }
            for (int edgeId : graph_.node(i).outEdgeIds_) {
                int coverId = cover2id(k, edgeId);
                constraint -= coverFlowVar_[coverId];
            }
            std::string constraintName = "FlowInOut[" + std::to_string(k) + std::to_string(i) + "]";
            coverModel_->addConstr(constraint == 0, constraintName);
       }
    }

    // Flow cover constraints
    for (int edgeId = 0; edgeId < edgeNum_; edgeId++) {
        GRBLinExpr constraint = - intFlowVar_[edgeId].get(GRB_DoubleAttr_X);
        for (int k = 0; k < intFlow_; k++) {
            int coverId = cover2id(k, edgeId);
            constraint += coverFlowVar_[coverId];
        }
        std::string constraintName = "EdgeFlowSum[" + std::to_string(edgeId) + "]";
        coverModel_->addConstr(constraint == 0, constraintName);
    }

    // Flow from the start node equals to 1
    int startId = graph_.node2id(startNode_);
    for (int k = 0; k < intFlow_; k++) {
        GRBLinExpr constraint = 0;
        for (int edgeId : graph_.node(startId).outEdgeIds_) { // for ( auto it = myset.begin(); it != myset.end(); ++it )
            int coverId = cover2id(k, edgeId);
            constraint += coverFlowVar_[coverId];
        }
        std::string constraintName = "PathFlow[" + std::to_string(k) + "]";
        coverModel_->addConstr(constraint == 1, constraintName);
    }

    // Minimize the maximum energy cost
    for (int k = 0; k < intFlow_; k++) {
        GRBLinExpr constraint = - vehVar_;
        for (int edgeId = 0; edgeId < edgeNum_; edgeId++) {
            int coverId = cover2id(k, edgeId);
            const GraphEdgeParam& edgeParam = graph_.edge(edgeId);
            constraint += edgeParam.engCost_ * coverFlowVar_[coverId];
        }
        std::string constraintName = "MaxEng[" + std::to_string(k) + "]";
        coverModel_->addConstr(constraint <= 0, constraintName);
    }
}

bool FlowConverter::getFlow() {
    if (!roundOptimized_) {
        return false;
    }
    int startId = graph_.node2id(startNode_);
    double tempIntFlow = 0.0;
    // oriFlow_ = 0.0;
    for (int edgeId : graph_.node(startId).outEdgeIds_) { // for ( auto it = myset.begin(); it != myset.end(); ++it )
        tempIntFlow += intFlowVar_[edgeId].get(GRB_DoubleAttr_X);
        // oriFlow_    += intFlowVar_[edgeId].get(GRB_DoubleAttr_LB);
    }
    intFlow_ = static_cast<int>(tempIntFlow + 0.5);
    std::cout << "Flow: " << oriFlow_ << " < " << tempIntFlow << " --> " << intFlow_ << "\n";
    return true;
}

bool FlowConverter::optimizeRound(double solverMaxTime) {
    // roundModel_->set(GRB_DoubleParam_TimeLimit, smallerTime);
    if (solverMaxTime > 0.001) {
        roundModel_->set(GRB_DoubleParam_TimeLimit, solverMaxTime);
    }
    roundModel_->optimize();
    roundOptimized_ = true;
    getFlow();
    return true;
}

bool FlowConverter::optimizeCover(double solverMaxTime) {
    if (solverMaxTime > 0.001) {
        coverModel_->set(GRB_DoubleParam_TimeLimit, solverMaxTime);
    }
    else {
        coverModel_->set(GRB_DoubleParam_TimeLimit, 1000.0);
    }
    coverModel_->optimize();
    coverOptimized_ = true;
    return true;
}

std::string FlowConverter::toStringRound(const std::string& prefix) const {
    if (!roundOptimized_) {
        return "";
    }
    std::ostringstream out;

    out << prefix << "roundOptimized: " << roundOptimized_ << "\n";
    out << prefix << "objVal: " << std::fixed << std::setprecision(6) << roundModel_->get(GRB_DoubleAttr_ObjVal) << "\n";
    out << prefix << "objBound: " << roundModel_->get(GRB_DoubleAttr_ObjBound) << "\n";
    // out << prefix << "MIPGap: " << roundModel_->get(GRB_DoubleAttr_MIPGap) << "\n";
    out << prefix << "varsNum: " << roundModel_->get(GRB_IntAttr_NumVars) << "\n";
    out << prefix << "constraintNum: " << roundModel_->get(GRB_IntAttr_NumConstrs) << "\n";
    out << prefix << "solCount: " << roundModel_->get(GRB_IntAttr_SolCount) << "\n";
    out << prefix << "nodeCount: " << roundModel_->get(GRB_DoubleAttr_NodeCount) << "\n";
    out << prefix << "iterCount: " << roundModel_->get(GRB_DoubleAttr_IterCount) << "\n";
    out << prefix << "solverTime: " << roundModel_->get(GRB_DoubleAttr_Runtime) << "\n";
    out << prefix << "avgObj: " << (roundModel_->get(GRB_DoubleAttr_ObjVal) / static_cast<double>(intFlow_) )<< "\n";

    out << prefix << "edgeNum: " << edgeNum_ << "\n";
    out << prefix << "nodeNum: " << nodeNum_ << "\n";
    out << prefix << "intFlow: " << intFlow_ << "\n";
    out << prefix << "oriFlow: " << oriFlow_ << "\n";
    out << prefix << "startNode: " << startNode_ << "\n";
    out << prefix << "endNode: " << endNode_ << "\n";

    size_t edgeNum = graph_.edgeNum();
    std::vector<double> oriFlowVec(edgeNum);
    for (size_t i = 0; i < edgeNum; i++) {
        oriFlowVec[i] = graph_.edge(i).timeCost_;
    }
    out << prefix << "oriFlowVar: " << vecToString(oriFlowVec) << "\n";

    std::vector<double> intFlowVec(edgeNum);
    for (size_t i = 0; i < edgeNum; i++) {
        intFlowVec[i] = intFlowVar_[i].get(GRB_DoubleAttr_X);
    }
    out << prefix << "intFlowVar: " << vecToString(intFlowVec) << "\n";

    return out.str();
}

void FlowConverter::printRound() const {
    std::cout << toStringRound("");
}

void FlowConverter::saveRound(const std::string& saveFileName, const std::string& prefix) const {
    std::ofstream logfile(saveFileName);
    logfile << toStringRound(prefix);
    logfile.close();
}

void FlowConverter::printCover() const {
    bool flagSave = false;
    std::cout << toStringCover(flagSave, "");
}

void FlowConverter::saveCover(const std::string& saveFileName, const std::string& prefix) const {
    bool flagSave = true;
    std::ofstream logfile(saveFileName);
    logfile << toStringCover(flagSave, prefix);
    logfile.close();
}

std::string FlowConverter::toStringCover(bool flagSave, const std::string& prefix) const {
    if (!coverOptimized_) {
        return "";
    }
    std::ostringstream out;

    std::vector<std::vector<int>> vehPath;
    std::vector<std::vector<int>> vehEdge;
    std::vector<double> vehSumEng;
    std::vector<double> vehSumVar;
    getPath(vehPath, vehEdge, vehSumEng, vehSumVar);

    out << prefix << "coverOptimized: " << coverOptimized_ << "\n";
    out << prefix << "objVal: " << std::fixed << std::setprecision(6) << coverModel_->get(GRB_DoubleAttr_ObjVal) << "\n";
    out << prefix << "objBound: " << coverModel_->get(GRB_DoubleAttr_ObjBound) << "\n";
    out << prefix << "MIPGap: " << coverModel_->get(GRB_DoubleAttr_MIPGap) << "\n";
    out << prefix << "varsNum: " << coverModel_->get(GRB_IntAttr_NumVars) << "\n";
    out << prefix << "constraintNum: " << coverModel_->get(GRB_IntAttr_NumConstrs) << "\n";
    out << prefix << "solCount: " << coverModel_->get(GRB_IntAttr_SolCount) << "\n";
    out << prefix << "nodeCount: " << coverModel_->get(GRB_DoubleAttr_NodeCount) << "\n";
    out << prefix << "iterCount: " << coverModel_->get(GRB_DoubleAttr_IterCount) << "\n";
    out << prefix << "solverTime: " << coverModel_->get(GRB_DoubleAttr_Runtime) << "\n";
    out << prefix << "maxEng: " << vecMax(vehSumEng) << "\n";
    double sumEng = vecSum(vehSumEng);
    double avgEng = sumEng / static_cast<double>(intFlow_);
    out << prefix << "avgEng: " << avgEng << "\n";
    out << prefix << "sumEng: " << sumEng << "\n";

    out << prefix << "edgeNum: " << edgeNum_ << "\n";
    out << prefix << "nodeNum: " << nodeNum_ << "\n";
    out << prefix << "intFlow: " << intFlow_ << "\n";
    out << prefix << "oriFlow: " << oriFlow_ << "\n";
    out << prefix << "startNode: " << startNode_ << "\n";
    out << prefix << "endNode: " << endNode_ << "\n";

    if (flagSave) {
        for (int k = 0; k < intFlow_; k++) {
            out << prefix << "v" << std::to_string(k+1) << ":\n";
            out << prefix << "  vehPath: " << vecToString(offsetString(0, vehPath[k], true)) << "\n";
            out << prefix << "  vehEdge: " << vecToString(offsetString(1, vehEdge[k], true)) << "\n";
            out << prefix << "  vehSumEng: " << vehSumEng[k] << "\n";
            out << prefix << "  vehSumVar: " << vehSumVar[k] << "\n";
        }
        // Save the raw result
        std::vector<double> coverFlowVec(intFlow_ * edgeNum_);
        for (int i = 0; i < intFlow_ * edgeNum_; i++) {
            coverFlowVec[i] = coverFlowVar_[i].get(GRB_DoubleAttr_X);
        }
        out << prefix << "coverFlowVar: " << vecToString(coverFlowVec) << "\n";
    }
    else {
        for (int k = 0; k < intFlow_; k++) {
            out << prefix << "v" << std::to_string(k+1) << ": " << vehSumEng[k] << ", " << vecToString(offsetString(0, vehPath[k], true), " -->") << "\n";
        }

    }

    return out.str();
}

void FlowConverter::getPath(std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue) const {
    if (!coverOptimized_) {
        return;
    }
    vehPath.clear();
    vehPath.resize(intFlow_);
    vehEdge.clear();
    vehEdge.resize(intFlow_);
    vehSumEng.clear();
    vehSumEng.resize(intFlow_);
    vehSumVar.clear();
    vehSumVar.resize(intFlow_);
    // Determine path for each vehicle
    for (int k = 0; k < intFlow_; k++) {
        vehPath[k].clear();
        vehEdge[k].clear();
        vehSumEng[k] = 0.0;
        vehSumVar[k] = 0.0;

        int startId = graph_.node2id(startNode_);
        int endId = graph_.node2id(endNode_);
        for (int i = startId; ;) {
            int travelEdgeId = -1;
            for (int edgeId : graph_.node(i).outEdgeIds_) {
                int xId = cover2id(k, edgeId);
                double x = 0.0;
                // Based on whether getPath is called in a callback function
                if (xValue == nullptr) {
                    x = coverFlowVar_[xId].get(GRB_DoubleAttr_X);
                }
                else {
                    x = xValue[xId];
                }
                if (x > 0.5) {
                    travelEdgeId = edgeId;
                    vehSumEng[k] += graph_.edge(edgeId).engCost_;
                    vehSumVar[k] += graph_.edge(edgeId).engUnc_;
                    break;
                }
            }
            if (travelEdgeId >= 0) {
                if (i == startId) {
                    vehPath[k].push_back(startNode_);
                }
                // int j = graph_.edge(travelEdgeId).edge_.node2_;
                // int nodej = graph_.node(j).node_;
                int nodej = graph_.edge(travelEdgeId).edge_.node2_;
                int j = graph_.node2id(nodej);
                vehPath[k].push_back(nodej);
                vehEdge[k].push_back(travelEdgeId);
                i = j;
            }
            else {
                break;
            }
        }
    }
}

std::string FlowConverter::offsetString(int type, int id, bool flagOffset) const {
    int offset = 0;
    if (flagOffset) {
        offset = 1;
    }
    std::string idName = "";
    if (type == 0) {
        // Node
        if (id == startNode_ || id == endNode_) {
            idName += "0";
        }
        else {
            idName += std::to_string(id + offset);
        }
    }
    else if (type == 1) {
        idName += std::to_string(id + offset);
    }
    return idName;
}

std::vector<int> FlowConverter::offsetString(int type, const std::vector<int>& id, bool flagOffset) const {
    int offset = 0;
    if (flagOffset) {
        offset = 1;
    }
    std::vector<int> idName = id;
    for (int i = 0; i < id.size(); i++) {
        if (type == 0) {
            // Node
            if (id[i] == startNode_ || id[i] == endNode_) {
                idName[i] = 0;
            }
            else {
                idName[i] += offset;
            }
        }
        else if (type == 1) {
            idName[i] += offset;
        }
    }
    return idName;
}

