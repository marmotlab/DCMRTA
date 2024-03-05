#include <TeamPlanner.hpp>

void TeamPlanner::formVarNameCost() {
    // Initialize variable names and costs
    for (int xId = 0; xId < xVarNum_; xId++) {
        GraphEdgeParam edgeParam;
        int veh;
        bool flagEdge = xId2edge(xId, veh, edgeParam);
        if (!flagEdge) {
            std::cout << "No edge!\n";
        }
        std::string varName = "x["  + toString(TEAMPLANNER_VEHC, veh)                    + ","
                                    + toString(TEAMPLANNER_NODE, edgeParam.edge_.node1_) + ","
                                    + toString(TEAMPLANNER_NODE, edgeParam.edge_.node2_) + ","
                                    + toString(TEAMPLANNER_PATH, edgeParam.edge_.type_)  + "]";
        xVar_[xId].set(GRB_DoubleAttr_Obj, edgeParam.engCost_);
        xVar_[xId].set(GRB_StringAttr_VarName, varName);
        // X is a continuous variable here
        if (flagContinuous_) {
            xVar_[xId].set(GRB_DoubleAttr_LB, 0.0);
            xVar_[xId].set(GRB_DoubleAttr_UB, static_cast<double>(param_.vehNumPerType_[veh]));
        }
    }

    for (int yId = 0; yId < yVarNum_; yId++) {
        int veh;
        int task;
        yId2sub(yId, veh, task);
        std::string varName = "y[" + toString(TEAMPLANNER_VEHC, veh)  + ","
                                   + toString(TEAMPLANNER_TASK, task) + "]";
        // yVar_[yId].set(GRB_DoubleAttr_Obj, 0.0);
        yVar_[yId].set(GRB_DoubleAttr_Obj, param_.vehParam_[veh].engCost_); // Strange, but corresponding to the MATLAB settings
        yVar_[yId].set(GRB_StringAttr_VarName, varName);
        if (flagContinuous_) {
            yVar_[yId].set(GRB_DoubleAttr_LB, 0.0);
            yVar_[yId].set(GRB_DoubleAttr_UB, static_cast<double>(param_.vehNumPerType_[veh]));
        }
    }

    if (flagContinuous_) {
        for (int xId = 0; xId < xVarNum_; xId++) {
            GraphEdgeParam edgeParam;
            int veh;
            bool flagEdge = xId2edge(xId, veh, edgeParam);
            if (!flagEdge) {
                std::cout << "No edge!\n";
            }
            std::string varName = "xr[" + toString(TEAMPLANNER_VEHC, veh)                    + ","
                                        + toString(TEAMPLANNER_NODE, edgeParam.edge_.node1_) + ","
                                        + toString(TEAMPLANNER_NODE, edgeParam.edge_.node2_) + ","
                                        + toString(TEAMPLANNER_PATH, edgeParam.edge_.type_)  + "]";
            xrVar_[xId].set(GRB_DoubleAttr_Obj, 0.0);
            xrVar_[xId].set(GRB_StringAttr_VarName, varName);
            // xVar_[xId].set(GRB_DoubleAttr_LB, 0.0);
            // xVar_[xId].set(GRB_DoubleAttr_UB, 1.0);
        }

        for (int yId = 0; yId < yVarNum_; yId++) {
            int veh;
            int task;
            yId2sub(yId, veh, task);
            std::string varName = "yr[" + toString(TEAMPLANNER_VEHC, veh)  + ","
                                    + toString(TEAMPLANNER_TASK, task) + "]";
            // yVar_[yId].set(GRB_DoubleAttr_Obj, 0.0);
            yrVar_[yId].set(GRB_DoubleAttr_Obj, 0.0);
            yrVar_[yId].set(GRB_StringAttr_VarName, varName);
            // yVar_[yId].set(GRB_DoubleAttr_LB, 0.0);
            // yVar_[yId].set(GRB_DoubleAttr_UB, 1.0);
        }

        for (int gId = 0; gId < gVarNum_; gId++) {
            int veh;
            int node;
            gId2sub(gId, veh, node);
            std::string varName = "g[" + toString(TEAMPLANNER_VEHC, veh)  + "," 
                                       + toString(TEAMPLANNER_NODE, node) + "]";
            gVar_[gId].set(GRB_DoubleAttr_Obj, 1.0 / 10000.0); // TODO, HARD CODE
            gVar_[gId].set(GRB_StringAttr_VarName, varName);
            gVar_[gId].set(GRB_DoubleAttr_LB, 0.0);
            gVar_[gId].set(GRB_DoubleAttr_UB, param_.MAXENG_);
        }
    }

    for (int zId = 0; zId < zVarNum_; zId++) {
        std::string varName = "z[" + toString(TEAMPLANNER_TASK, zId) + "]";
        zVar_[zId].set(GRB_DoubleAttr_Obj, 0.0);
        zVar_[zId].set(GRB_StringAttr_VarName, varName);
        // zVar_[zId].set(GRB_DoubleAttr_LB, 0.0);
        // zVar_[zId].set(GRB_DoubleAttr_UB, 1.0);
    }

    for (int qId = 0; qId < qVarNum_; qId++) {
        double varCost = 0.0;
        std::string varName = "q[";
        if (qId < taskNum_) {
            varName += toString(TEAMPLANNER_NODE, qId);
            varCost = param_.timePenalty1_;
        }
        else if (qId < taskNum_ + vehNum_) {
            varName += toString(TEAMPLANNER_NODE, qId);
            varCost = param_.timePenalty1_;
        }
        else {
            varName += toString(TEAMPLANNER_NODE, qId);
            varCost = param_.timePenalty_;
        }
        varName += "]";
        qVar_[qId].set(GRB_DoubleAttr_Obj, varCost);
        qVar_[qId].set(GRB_StringAttr_VarName, varName);
        qVar_[qId].set(GRB_DoubleAttr_LB, 0.0);
        qVar_[qId].set(GRB_DoubleAttr_UB, param_.MAXTIME_);
    }

    if (param_.flagSolver_ == TEAMPLANNER_SPR || param_.flagSolver_ == TEAMPLANNER_SPRITER) {
        for (int thetaId = 0; thetaId < vehNum_; thetaId++) {
            std::string varName = "theta[" + toString(TEAMPLANNER_VEHC, thetaId) + "]";
            thetaVar_[thetaId].set(GRB_DoubleAttr_Obj, param_.recoursePenalty_);
            thetaVar_[thetaId].set(GRB_StringAttr_VarName, varName);
            thetaVar_[thetaId].set(GRB_DoubleAttr_LB, 0.0);
            // thetaVar_[thetaId].set(GRB_DoubleAttr_UB, 1.0); // Default upper bound 1e+100
        }
    }

}

void TeamPlanner::formCommonModel() {
    model_->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Flow Constraint: incoming edges == outgoing edges
    for (int k = 0; k < vehNum_; k++) {
        for (int i = 0; i < taskNum_; i++) {
            GRBLinExpr constraint = 0;
            for (int edgeId : graph_[k].node(i).inEdgeIds_) { // for ( auto it = myset.begin(); it != myset.end(); ++it )
                int xId = sub2xId(k, edgeId);
                constraint += xVar_[xId];
            }
            for (int edgeId : graph_[k].node(i).outEdgeIds_) {
                int xId = sub2xId(k, edgeId);
                constraint -= xVar_[xId];
            }
            std::string constraintName = "FlowInOut[" + toString(TEAMPLANNER_VEHC, k) + "," + toString(TEAMPLANNER_NODE, i) + "]";
            model_->addConstr(constraint == 0, constraintName);
        }
    }

    // Flow Constraint: incoming edges <= 1
    for (int k = 0; k < vehNum_; k++) {
        for (int i = 0; i < taskNum_; i++) {
            GRBLinExpr constraint = 0;
            for (int edgeId : graph_[k].node(i).inEdgeIds_) {
                int xId = sub2xId(k, edgeId);
                constraint += xVar_[xId];
            }
            std::string constraintName = "FlowLessOne[" + toString(TEAMPLANNER_VEHC, k) + "," + toString(TEAMPLANNER_NODE, i) + "]";
            if (flagContinuous_) {
                model_->addConstr(constraint <= static_cast<double>(param_.vehNumPerType_[k]), constraintName);
            }
            else {
                model_->addConstr(constraint <= 1, constraintName);
            }
        }
    }

    // Flow Constraint: Outgoing edge from start <= 1
    for (int k = 0; k < vehNum_; k++) {
        int s = taskNum_ + k;
        GRBLinExpr constraint = 0;
        for (int edgeId : graph_[k].node(s).outEdgeIds_) {
            int xId = sub2xId(k, edgeId);
            constraint += xVar_[xId];
        }
        std::string constraintName = "FlowLessOne[" + toString(TEAMPLANNER_VEHC, k) + "," + toString(TEAMPLANNER_NODE, s) + "]";
        if (flagContinuous_) {
            model_->addConstr(constraint <= static_cast<double>(param_.vehNumPerType_[k]), constraintName);
        }
        else {
            model_->addConstr(constraint <= 1, constraintName);
        }
    }

    // Flow Constraint: Incoming edge <= outgoing edge from start (redundant but useful for solver)
    for (int k = 0; k < vehNum_; k++) {
        int s = taskNum_ + k;
        GRBLinExpr constraint0 = 0;
        for (int edgeId : graph_[k].node(s).outEdgeIds_) {
            int xId = sub2xId(k, edgeId);
            constraint0 -= xVar_[xId];
        }
        for (int i = 0; i < taskNum_; i++) {
            GRBLinExpr constraint = constraint0;
            for (int edgeId : graph_[k].node(i).inEdgeIds_) {
                int xId = sub2xId(k, edgeId);
                constraint += xVar_[xId];
            }
            std::string constraintName = "FlowLessStart[" + toString(TEAMPLANNER_VEHC, k) + "," + toString(TEAMPLANNER_NODE, s) + "]";
            model_->addConstr(constraint <= 0, constraintName);
        }
    }

    // Variable Relationship Constraint: Incoming edges == y
    for (int k = 0; k < vehNum_; k++) {
        for (int i = 0; i < taskNum_; i++) {
            GRBLinExpr constraint = 0;
            for (int edgeId : graph_[k].node(i).inEdgeIds_) {
                int xId = sub2xId(k, edgeId);
                constraint += xVar_[xId];
            }
            int yId = sub2yId(k, i);
            constraint -= yVar_[yId];
            std::string constraintName = "RelationXY[" + toString(TEAMPLANNER_VEHC, k) + "," + toString(TEAMPLANNER_NODE, i) + "]";
            model_->addConstr(constraint == 0, constraintName);
        }
    }

    // Time Constraint: Edge time
    for (int k = 0; k < vehNum_; k++) {
        for (int edgeId = 0; edgeId < static_cast<int>(graph_[k].edgeNum()); edgeId++) {
            const GraphEdgeParam& an_edge = graph_[k].edge(edgeId);
            // const 
            int i = an_edge.edge_.node1_;
            int j = an_edge.edge_.node2_;
            int xId = sub2xId(k, edgeId);
            if (i < 0) {
                // This edge does not exist
                continue;
            }
            GRBVar tempBinaryXVar;
            if (flagContinuous_) {
                tempBinaryXVar = xrVar_[xId];
            }
            else {
                tempBinaryXVar = xVar_[xId];
            }
            GRBLinExpr constraint = qVar_[i] - qVar_[j] + graph_[k].node(i).timeCost_ + an_edge.timeCost_ - param_.LARGETIME_ * (1 - tempBinaryXVar);
            std::string constraintName = "TimeEdge[" + toString(TEAMPLANNER_VEHC, k)                    + ","
                                                     + toString(TEAMPLANNER_NODE, an_edge.edge_.node1_) + ","
                                                     + toString(TEAMPLANNER_NODE, an_edge.edge_.node2_) + ","
                                                     + toString(TEAMPLANNER_PATH, an_edge.edge_.type_)  + "]";
            model_->addConstr(constraint <= 0, constraintName);
        }
    }

    // Time Constraint: Start time
    for (int k = 0; k < vehNum_; k++) {
        int s = taskNum_ + k;
        GRBLinExpr constraint = qVar_[s];
        std::string constraintName = "TimeStart[" + toString(TEAMPLANNER_NODE, s) + "]";
        model_->addConstr(constraint == 0, constraintName);
    }

    // Task Complete Constraint
    if (param_.flagTaskComplete_) {
        for (int i = 0; i < taskNum_; i++) {
            GRBLinExpr constraint = zVar_[i];
            std::string constraintName = "TaskComplete[" + toString(TEAMPLANNER_TASK, i) + "]";
            model_->addConstr(constraint == 1, constraintName);
        }
    }
    else {
        for (int i = 0; i < taskNum_; i++) {
            zVar_[i].set(GRB_DoubleAttr_Obj, -param_.taskCompleteReward_);
        }
    }

    // Task Requirements Constraint
    for (int i = 0; i < taskNum_; i++) {
        std::vector<int> requiredCaps;
        for (int andId = 0; andId < static_cast<int>(param_.taskParam_[i].reqFcn_.size()); andId++) {
            GRBLinExpr constraint3 = zVar_[i];
            for (int orId = 0; orId < static_cast<int>(param_.taskParam_[i].reqFcn_[andId].size()); orId++) {
                const TaskReqGeq& aReq = param_.taskParam_[i].reqFcn_[andId][orId];
                requiredCaps.push_back(aReq.capId_);
                // Add alpha variable
                double varLB = 0.0;
                double varUB = sumCap_[aReq.capId_];
                double varObj = 0.0;
                std::string commonName  = toString(TEAMPLANNER_TASK, i)     + ","
                                        + toString(TEAMPLANNER_AND , andId) + ","
                                        + toString(TEAMPLANNER_OR  , orId)  + "]";
                std::string varName = "alpha[" + commonName;
                GRBVar alphaVar = model_->addVar(varLB, varUB, varObj, GRB_CONTINUOUS, varName);
                alphaVar_.push_back(alphaVar);
                // Task Requirement Constraint: alpha definition
                if (param_.capType_[aReq.capId_] == 1) {
                    // Non-cumulative capability
                    GRBLinExpr constraint0 = alphaVar;
                    for (int k = 0; k < vehNum_; k++) {
                        int yId = sub2yId(k, i);
                        GRBVar tempBinaryYVar;
                        if (flagContinuous_) {
                            tempBinaryYVar = yrVar_[yId];
                        }
                        else {
                            tempBinaryYVar = yVar_[yId];
                        }
                        std::string constraintNameAlpha = "TaskReqAlphaNonCumu[" + commonName;
                        model_->addConstr(alphaVar <= varUB * (1 - tempBinaryYVar) + param_.vehParam_[k].capVector_[aReq.capId_] * tempBinaryYVar, constraintNameAlpha);
                        if (param_.vehParam_[k].capVector_[aReq.capId_] > 0.1) {
                            constraint0 -= param_.vehParam_[k].capVector_[aReq.capId_] * tempBinaryYVar;
                        }
                    }
                    std::string constraintName0 = "TaskReqAlphaDef[" + commonName;
                    model_->addConstr(constraint0 <= 0, constraintName0);
                }
                else {
                    // Cumulative capability
                    GRBLinExpr constraint0 = alphaVar;
                    for (int k = 0; k < vehNum_; k++) {
                        int yId = sub2yId(k, i);
                        if (param_.vehParam_[k].capVector_[aReq.capId_] > 0.1) {
                            constraint0 -= param_.vehParam_[k].capVector_[aReq.capId_] * yVar_[yId];
                        }
                    }
                    std::string constraintName0 = "TaskReqAlphaDef[" + commonName;
                    model_->addConstr(constraint0 == 0, constraintName0);
                }

                // Add w variable
                varLB = 0.0;
                varUB = 1.0;
                varObj = 0.0;
                varName = "w[" + commonName;
                GRBVar wVar = model_->addVar(varLB, varUB, varObj, GRB_BINARY, varName); // w should be int variables
                wVar_.push_back(wVar);
                constraint3 -= wVar;
                // Task Requirement Constraint: Relationship between w and alpha
                GRBLinExpr wVarExpress = wVar;
                // if (!aReq.geq_) {
                //     wVarExpress = 1 - wVar;
                // }
                double tempLargeCap = sumCap_[aReq.capId_] - aReq.capReq_ + 1.0;
                if (tempLargeCap < 1.0) {
                    tempLargeCap = 1.0;
                }
                GRBLinExpr constraint1 = aReq.capReq_ * wVarExpress - alphaVar;
                if (!aReq.geq_) {
                    constraint1 = aReq.capReq_+1 - (aReq.capReq_+1) * wVarExpress - alphaVar;
                }
                std::string constraintName1 = "TaskReqOrL[" + commonName;
                model_->addConstr(constraint1 <= 0, constraintName1);
                GRBLinExpr constraint2 = -tempLargeCap * wVarExpress + alphaVar - aReq.capReq_ + 1;
                if (!aReq.geq_) {
                    constraint2 = tempLargeCap * wVarExpress - tempLargeCap + alphaVar - aReq.capReq_;
                }
                std::string constraintName2 = "TaskReqOrG[" + commonName;
                model_->addConstr(constraint2 <= 0, constraintName2);
            }
            // Task Requirement Constraint: and
            std::string constraintName3 = "TaskReqAnd[" + toString(TEAMPLANNER_TASK, i)     + ","
                                                        + toString(TEAMPLANNER_AND , andId) + "]";
            model_->addConstr(constraint3 <= 0, constraintName3);
        }
        if (param_.flagNotUseUnralavant_) {
            // Task Requirement Constraint: Not use unrelavant vehicles
            for (int k = 0; k < vehNum_; k++) {
                bool flagUnrelavant = true;
                for (int capId : requiredCaps) {
                    if (param_.vehParam_[k].capVector_[capId] > 0.1) {
                        flagUnrelavant = false;
                        break;
                    }
                }
                if (flagUnrelavant) {
                    int yId = sub2yId(k, i);
                    std::string constraintName4 = "TaskReqUnrelavant["  + toString(TEAMPLANNER_TASK, i) + ","
                                                                        + toString(TEAMPLANNER_VEHC, k) + "]";
                    model_->addConstr(yVar_[yId] == 0, constraintName4);
                }
            }
        }
    }
}

void TeamPlanner::formDeterministicEnergy() {
    for (int k = 0; k < vehNum_; k++) {
        GRBLinExpr constraint = 0;
        for (int edgeId = 0; edgeId < static_cast<int>(graph_[k].edgeNum()); edgeId++) {
            const GraphEdgeParam& an_edge = graph_[k].edge(edgeId);
            int i = an_edge.edge_.node1_;
            if (i < 0) {
                // This edge does not exist
                continue;
            }
            int xId = sub2xId(k, edgeId);
            constraint += an_edge.engCost_ * xVar_[xId];
        }
        std::string constraintName = "ExpectedEng[" + toString(TEAMPLANNER_VEHC, k) + "]";
        // GRBConstr aConstraint = 
        model_->addConstr(constraint <= param_.vehParam_[k].engCap_, constraintName);
        // aConstraint.set(GRB_IntAttr_Lazy, 0);
    }
}
