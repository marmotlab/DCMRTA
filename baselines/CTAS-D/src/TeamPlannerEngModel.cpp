#include <TeamPlanner.hpp>

void TeamPlanner::formCCPEnergy() {
    // for (int k = 0; k < vehNum_; k++) {
    //     GRBQuadExpr muExpr = param_.vehParam_[k].engCap_;
    //     GRBQuadExpr varExpr = 0;
    //     double betaRelated = normalCDFInverse(param_.CcpBeta_);
    //     betaRelated *= betaRelated;
    //     for (int edgeId = 0; edgeId < static_cast<int>(graph_[k].edgeNum()); edgeId++) {
    //         const GraphEdgeParam& an_edge = graph_[k].edge(edgeId);
    //         int i = an_edge.edge_.node1_;
    //         if (i < 0) {
    //             // This edge does not exist
    //             continue;
    //         }
    //         int xId = sub2xId(k, edgeId);
    //         muExpr -= an_edge.engCost_ * xVar_[xId];
    //         varExpr += betaRelated * an_edge.engUnc_ * xVar_[xId];
    //     }
    //     std::string constraintName = "CCPEng[" + toString(TEAMPLANNER_VEHC, k) + "]";
    //     model_->addQConstr(varExpr <= muExpr * muExpr, constraintName);
    //     // model_->addConstr(varExpr <= muExpr * muExpr, constraintName);
    // }
    for (int k = 0; k < vehNum_; k++) {
        double engCap = param_.vehParam_[k].engCap_;
        GRBQuadExpr muExpr = engCap;
        GRBQuadExpr varExpr = 0;
        double betaRelated = normalCDFInverse(param_.CcpBeta_);
        betaRelated = betaRelated * betaRelated / engCap;
        for (int edgeId = 0; edgeId < static_cast<int>(graph_[k].edgeNum()); edgeId++) {
            const GraphEdgeParam& an_edge = graph_[k].edge(edgeId);
            if (an_edge.edge_.node1_ < 0) {
                continue; // This edge does not exist
            }
            int xId = sub2xId(k, edgeId);
            for (int edgeId2 = 0; edgeId2 < static_cast<int>(graph_[k].edgeNum()); edgeId2++) {
                const GraphEdgeParam& an_edge2 = graph_[k].edge(edgeId2);
                if (an_edge2.edge_.node1_ < 0) {
                    continue; // This edge does not exist
                }
                int xId2 = sub2xId(k, edgeId2);
                muExpr += an_edge.engCost_ * an_edge2.engCost_ / engCap * xVar_[xId] * xVar_[xId2];
            }
            muExpr -= 2 * an_edge.engCost_ * xVar_[xId];
            varExpr += betaRelated * an_edge.engUnc_ * xVar_[xId];
        }
        std::string constraintName = "CCPEng[" + toString(TEAMPLANNER_VEHC, k) + "]";
        model_->addQConstr(varExpr <= muExpr, constraintName);
        // model_->addConstr(varExpr <= muExpr * muExpr, constraintName);
    }
}

bool TeamPlanner::addRecourseCut(double* xValue, double* thetaValue, std::vector<GRBLinExpr>& recourseCut, std::vector<double>& vehRecourseCost) const {
    getRecourseCost(vehRecourseCost, xValue);
    for (int k = 0; k < vehNum_; k++) {
        double gk = vehRecourseCost[k];
        double thetak = 0.0;
        if (thetaValue == nullptr) {
            thetak = thetaVar_[k].get(GRB_DoubleAttr_X);
        }
        else {
            thetak = thetaValue[k];
        }
        if (thetak > gk - 0.01) { // Hardcode 0.01
            continue;
        }
        // The if here is actually redundant, we can always choose else
        if (!param_.flagSprAddCutToSameType_) {
            GRBLinExpr aCut =  - thetaVar_[k] + gk;
            for (int edgeId = 0; edgeId < edgeNum_; edgeId++) {
                int xId = sub2xId(k, edgeId);
                double x = 0.0;
                // Based on whether getPath is called in a callback function
                if (xValue == nullptr) {
                    x = xVar_[xId].get(GRB_DoubleAttr_X);
                }
                else {
                    x = xValue[xId];
                }
                if (x > 0.5) {
                    aCut +=  gk * (xVar_[xId] - 1);
                }
            }
            recourseCut.push_back(aCut);
        }
        else {
            int repeatNum = 1;
            int kSameVeh = k;
            if (param_.flagSprAddCutToSameType_) {
                repeatNum = vehNum_ / vehTypeNum_;
                kSameVeh = k % vehTypeNum_;
                // printf("repeatNum, kSameVeh, k = %d, %d, %d \n", repeatNum, kSameVeh, k);
            }
            std::vector<int> edgeIdList;
            for (int edgeId = 0; edgeId < edgeNum_; edgeId++) {
                int xId = sub2xId(k, edgeId);
                double x = 0.0;
                // Based on whether getPath is called in a callback function
                if (xValue == nullptr) {
                    x = xVar_[xId].get(GRB_DoubleAttr_X);
                }
                else {
                    x = xValue[xId];
                }
                if (x > 0.5) {
                    edgeIdList.push_back(edgeId);
                    // aCut +=  gk * (xVar_[xId] - 1);
                }
            }
            for (int repeatId = 0; repeatId < repeatNum; repeatId++) {
                int tempK = k;
                if (repeatNum > 1) {
                    tempK = kSameVeh + repeatId * vehTypeNum_;
                }
                GRBLinExpr aCut =  - thetaVar_[tempK] + gk;
                for (int edgeId : edgeIdList) {
                    int xId = sub2xId(tempK, edgeId);
                    aCut +=  gk * (xVar_[xId] - 1);
                }
                recourseCut.push_back(aCut);
            }
        }
    }
    return (recourseCut.size() > 0);
}

bool TeamPlanner::getRecourseCost(std::vector<double>& vehRecourseCost, const double* xValue) const {
    bool flagRecourse = false;
    vehRecourseCost.clear();
    vehRecourseCost.resize(vehNum_);
    std::vector<std::vector<int>> vehPath;
    std::vector<std::vector<int>> vehEdge;
    std::vector<double> vehSumEng;
    std::vector<double> vehSumVar;
    getPath(vehPath, vehEdge, vehSumEng, vehSumVar, xValue);
    for (int k = 0; k < vehNum_; k++) {
        vehRecourseCost[k] = 0;
        int pathLen = static_cast<int>(vehEdge[k].size());
        if (pathLen > 0 && (pathLen+1) != static_cast<int>(vehPath[k].size()) ) {
            // printf("(%d, %d)", pathLen, static_cast<int>(vehPath[k].size()));
            fprintf(stderr, "ERROR: Edge and node number do not match!\n");
        }
        double prevSumEdgeCost = 0.0;
        double prevSumEdgeVar = 0.0;
        double prevRescueCost = 0.0;
        // double prevReplaceCost = 0.0;
        int lNum = 3;
        std::vector<double> prevProb(lNum);
        std::fill(prevProb.begin(), prevProb.end(), 1.0);
        for (int p = 0; p < pathLen; p++) {
            const GraphEdgeParam& anEdge = graph_[k].edge(vehEdge[k][p]);
            double sumEdgeCost = prevSumEdgeCost + anEdge.engCost_;
            double sumEdgeVar = prevSumEdgeVar + anEdge.engUnc_;
            double sumEdgeStd = std::sqrt(sumEdgeVar);
            
            int replaceEdgeId = graph_[k].edge2id(taskNum_ + k, anEdge.edge_.node2_, 0);
            double replaceCost = 0.0;
            if (replaceEdgeId >= 0) {
                replaceCost = graph_[k].edge(replaceEdgeId).engCost_;
            }
            // TODO: to change this 0 to a real rescue vehicle
            int rescueVehId = 0;
            int rescueComeId = graph_[rescueVehId].edge2id(taskNum_ + rescueVehId, anEdge.edge_.node2_, 0);
            int rescueBackId = graph_[rescueVehId].edge2id(anEdge.edge_.node2_, taskNum_ + vehNum_ + rescueVehId, 0);
            double currRescueCost = 0.0;
            if (rescueComeId >= 0 && rescueBackId >= 0) {
                currRescueCost = graph_[rescueVehId].edge(rescueComeId).engCost_ + graph_[rescueVehId].edge(rescueBackId).engCost_;
            }
            // printf("replaceEdgeId = %d, rescueComeId = %d, rescueBackId = %d\n", replaceEdgeId, rescueComeId, rescueBackId);
            double rescueCost = (prevRescueCost + currRescueCost) / 2.0;

            double tempRecourseCost = rescueCost + replaceCost;
            for (int l = 0; l < lNum; l++) {
                double currProb = normalCDF( static_cast<double>(l+1) * param_.vehParam_[k].engCap_, sumEdgeCost, sumEdgeStd);
                vehRecourseCost[k] += (prevProb[l] - currProb) * tempRecourseCost;
                // printf("haha = %f, %f, %f, %f, %f\n", prevProb[l] - currProb, (prevProb[l] - currProb) * tempRecourseCost, tempRecourseCost, rescueCost, replaceCost);
                prevProb[l] = currProb;
            }
            prevSumEdgeCost = sumEdgeCost;
            prevSumEdgeVar = sumEdgeVar;
            prevRescueCost = currRescueCost;
            // prevReplaceCost = replaceCost;
        }
    }
    return flagRecourse;
}

bool TeamPlanner::addCCPCut(double* xValue, std::vector<GRBLinExpr>& ccpCut) const {
    std::vector<std::vector<int>> vehPath;
    std::vector<std::vector<int>> vehEdge;
    std::vector<double> vehSumEng;
    std::vector<double> vehSumVar;
    getPath(vehPath, vehEdge, vehSumEng, vehSumVar, xValue);
    for (int k = 0; k < vehNum_; k++) {
        double betaRelated = normalCDFInverse(param_.CcpBeta_);
        if (vehSumEng[k] + betaRelated * sqrt(vehSumVar[k]) <= param_.vehParam_[k].engCap_) {
            continue;
        }
        printf("mu, sigma, beta, mu+sigma, cap = %f, %f, %f, %f, %f\n", vehSumEng[k], sqrt(vehSumVar[k]), betaRelated, vehSumEng[k] + betaRelated * sqrt(vehSumVar[k]), param_.vehParam_[k].engCap_);
        if (param_.flagSprAddCutToSameType_) {
            int repeatNum = vehNum_ / vehTypeNum_;
            int kSameVeh = k % vehTypeNum_;
            for (int repeatId = 0; repeatId < repeatNum; repeatId++) {
                int tempK = kSameVeh + repeatId * vehTypeNum_;
                GRBLinExpr aCut = 1;
                for (int edgeId : vehEdge[k]) {
                    int xId = sub2xId(tempK, edgeId);
                    aCut += xVar_[xId] - 1;
                }
                ccpCut.push_back(aCut);
            }
        }
        else {
            GRBLinExpr aCut = 1;
            for (int edgeId : vehEdge[k]) {
                int xId = sub2xId(k, edgeId);
                aCut += xVar_[xId] - 1;
            }
            ccpCut.push_back(aCut);
        }
    }
    return (ccpCut.size() > 0);
}
