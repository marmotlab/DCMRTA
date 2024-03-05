#include <TeamPlanner.hpp>

void TeamPlanner::formContinuousModel() {
    for (int k = 0; k < vehNum_; k++) {
        for (int edgeId = 0; edgeId < static_cast<int>(graph_[k].edgeNum()); edgeId++) {
            int xId = sub2xId(k, edgeId);
            const GraphEdgeParam& an_edge = graph_[k].edge(edgeId);
            std::string commonName =  toString(TEAMPLANNER_VEHC, k)                    + ","
                                    + toString(TEAMPLANNER_NODE, an_edge.edge_.node1_) + ","
                                    + toString(TEAMPLANNER_NODE, an_edge.edge_.node2_) + ","
                                    + toString(TEAMPLANNER_PATH, an_edge.edge_.type_)  + "]";
            std::string constraintName = "X_Xr1[" + commonName;
            model_->addConstr(xVar_[xId] >= xrVar_[xId], constraintName);
            constraintName = "X_Xr2[" + commonName;
            model_->addConstr(xVar_[xId] <= xrVar_[xId] * static_cast<double>(param_.vehNumPerType_[k]), constraintName);
        }
    }
    for (int k = 0; k < vehNum_; k++) {
        for (int i = 0; i < taskNum_; i++) {
            int yId = sub2yId(k, i);
            std::string commonName = toString(TEAMPLANNER_VEHC, k) + "," + toString(TEAMPLANNER_NODE, i) + "]";
            std::string constraintName = "Y_Yr1[" + commonName;
            model_->addConstr(yVar_[yId] >= yrVar_[yId], constraintName);
            constraintName = "Y_Yr2[" + commonName;
            model_->addConstr(yVar_[yId] <= yrVar_[yId] * static_cast<double>(param_.vehNumPerType_[k]), constraintName);
        }
    }
}

void TeamPlanner::formContinuousDetEnergy() {
    for (int k = 0; k < vehNum_; k++) {
        for (int edgeId = 0; edgeId < static_cast<int>(graph_[k].edgeNum()); edgeId++) {
            const GraphEdgeParam& an_edge = graph_[k].edge(edgeId);
            if (an_edge.edge_.node1_ < 0) {
                // This edge does not exist
                continue;
            }
            int xId = sub2xId(k, edgeId);
            int i = sub2gId(k, an_edge.edge_.node1_);
            int j = sub2gId(k, an_edge.edge_.node2_);
            GRBLinExpr constraint = gVar_[i] - gVar_[j] + an_edge.engCost_ - param_.MAXENG_ * (1 - xrVar_[xId]);
            // GRBLinExpr constraint = gVar_[i] - gVar_[j] + an_edge.engCost_ * xrVar_[xId] - param_.MAXENG_ * (1 - xrVar_[xId]);
            std::string constraintName = "EngEdge[" + toString(TEAMPLANNER_VEHC, k)                    + ","
                                                     + toString(TEAMPLANNER_NODE, an_edge.edge_.node1_) + ","
                                                     + toString(TEAMPLANNER_NODE, an_edge.edge_.node2_) + ","
                                                     + toString(TEAMPLANNER_PATH, an_edge.edge_.type_)  + "]";
            model_->addConstr(constraint <= 0, constraintName);
        }
    }
    for (int gId = 0; gId < gVarNum_; gId++) {
        int k;
        int i;
        gId2sub(gId, k, i);
        std::string constraintName = "EngNode[" + toString(TEAMPLANNER_VEHC, k) + ","
                                                + toString(TEAMPLANNER_NODE, i);
        if (i == k+taskNum_) {
            model_->addConstr(gVar_[gId] == 0, constraintName);
        }
        else {
            model_->addConstr(gVar_[gId] <= param_.vehParam_[k].engCap_, constraintName);
        }
    }
}