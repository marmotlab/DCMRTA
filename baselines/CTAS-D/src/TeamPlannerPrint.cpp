#include <TeamPlanner.hpp>
#include <FlowConverter.hpp>

std::string TeamPlanner::toString(SymbolType type, int id, bool flagOffset, bool flagNumberOnly) const {
    int offset = 0;
    if (flagOffset) {
        offset = 1;
    }
    std::string idName = "";
    if (flagNumberOnly) {
        switch (type)
        {
        case TEAMPLANNER_VEHC:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_NODE:
            if (id < taskNum_) {
                idName += std::to_string(id + offset);
            }
            else if (id < taskNum_ + vehNum_) {
                idName += "0";
            }
            else {
                idName += "0";
            }
            break;
        case TEAMPLANNER_TASK:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_PATH:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_AND:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_OR:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_VEHCTYPE:
            idName += std::to_string(id%vehTypeNum_ + offset);
            break;
        default:
            break;
        }
    }
    else {
        switch (type)
        {
        case TEAMPLANNER_VEHC:
            idName += "v" + std::to_string(id + offset);
            break;
        case TEAMPLANNER_NODE:
            if (id < taskNum_) {
                idName += "m" + std::to_string(id + offset);
            }
            else if (id < taskNum_ + vehNum_) {
                idName += "s" + std::to_string(id - taskNum_ + offset);
            }
            else {
                idName += "u" + std::to_string(id - vehNum_ - taskNum_ + offset);
            }
            break;
        case TEAMPLANNER_TASK:
            idName += "m" + std::to_string(id + offset);
            break;
        case TEAMPLANNER_PATH:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_AND:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_OR:
            idName += std::to_string(id + offset);
            break;
        case TEAMPLANNER_VEHCTYPE:
            idName += "v" + std::to_string(id%vehTypeNum_ + offset);
            break;
        default:
            break;
        }
    }
    return idName;
}

void TeamPlanner::getCost(const std::vector<double>& vehRecourseCost,
                        double& energyCost, double& timeCost, double& otherCost, 
                        double& fCost, double& thetaCost, double& recourseCost,
                        double& oriLinCost, double& oriNonCost,
                        double& oriRiskLinCost, double& oriRiskNonCost) {
    energyCost = 0.0;
    timeCost = 0.0;
    recourseCost = 0.0;
    otherCost = 0.0;
    fCost = 0.0;
    thetaCost = 0.0;
    for (int i = 0; i < xVarNum_; i++) {
        energyCost += xVar_[i].get(GRB_DoubleAttr_X) * xVar_[i].get(GRB_DoubleAttr_Obj);
    }
    // printf("getCost: xVarCost = %f\n", energyCost);
    double otherCostPrev = otherCost;
    for (int i = 0; i < yVarNum_; i++) {
        otherCost += yVar_[i].get(GRB_DoubleAttr_X) * yVar_[i].get(GRB_DoubleAttr_Obj);
    }
    // printf("getCost: yVarCost = %f\n", otherCost - otherCostPrev);
    otherCostPrev = otherCost;
    for (int i = 0; i < zVarNum_; i++) {
        otherCost += zVar_[i].get(GRB_DoubleAttr_X) * zVar_[i].get(GRB_DoubleAttr_Obj);
    }
    // printf("getCost: zVarCost = %f\n", otherCost - otherCostPrev);
    otherCostPrev = otherCost;
    for (size_t i = 0; i < alphaVar_.size(); i++) {
        otherCost += alphaVar_[i].get(GRB_DoubleAttr_X) * alphaVar_[i].get(GRB_DoubleAttr_Obj);
    }
    // printf("getCost: alphaVarCost = %f\n", otherCost - otherCostPrev);
    otherCostPrev = otherCost;
    for (size_t i = 0; i < wVar_.size(); i++) {
        otherCost += wVar_[i].get(GRB_DoubleAttr_X) * wVar_[i].get(GRB_DoubleAttr_Obj);
    }
    // printf("getCost: wVarCost = %f\n", otherCost - otherCostPrev);
    otherCostPrev = otherCost;
    for (int i = 0; i < qVarNum_; i++) {
        timeCost += qVar_[i].get(GRB_DoubleAttr_X) * qVar_[i].get(GRB_DoubleAttr_Obj);
//        printf(" %f", qVar_[i].get(GRB_DoubleAttr_X) );
//        printf(" %f", qVar_[i].get(GRB_DoubleAttr_Obj) );
//        printf(" %f", timeCost);
    }
    // printf("getCost: qVarCost = %f\n", timeCost);

    if (thetaVar_ != nullptr) {
        for (int i = 0; i < vehNum_; i++) {
            thetaCost += thetaVar_[i].get(GRB_DoubleAttr_X) * thetaVar_[i].get(GRB_DoubleAttr_Obj);
        }
    }
    // printf("getCost: thetaVarCost = %f\n", thetaCost);
    for (size_t i = 0; i < vehRecourseCost.size(); i++) {
        recourseCost += vehRecourseCost[i];
    }
    // printf("getCost: recourseCost = %f\n", recourseCost);

    if (xrVar_ != nullptr) {
        otherCostPrev = otherCost;
        for (int i = 0; i < xVarNum_; i++) {
            otherCost += xrVar_[i].get(GRB_DoubleAttr_X) * xrVar_[i].get(GRB_DoubleAttr_Obj);
        }
        // printf("getCost: xrVarCost = %f\n", otherCost - otherCostPrev);
    }
    if (yrVar_ != nullptr) {
        otherCostPrev = otherCost;
        for (int i = 0; i < yVarNum_; i++) {
            otherCost += yrVar_[i].get(GRB_DoubleAttr_X) * yrVar_[i].get(GRB_DoubleAttr_Obj);
        }
        // printf("getCost: yrVarCost = %f\n", otherCost - otherCostPrev);
    }
    double flowEnergyCost = 0.0;
    if (gVar_ != nullptr) {
        for (int i = 0; i < gVarNum_; i++) {
            flowEnergyCost += gVar_[i].get(GRB_DoubleAttr_X) * gVar_[i].get(GRB_DoubleAttr_Obj);
        }
        // printf("getCost: flowEnergyCost = %f\n", flowEnergyCost);
    }

    // double riskCostLin = 0.0;
    // if (flagTaskVarAdded_) {
    //     for (int i = 0; i < taskNum_; i++) {
    //         for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
    //             for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
    //                 const TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
    //                 riskCostLin += aGeq.riskVar_.get(GRB_DoubleAttr_X) * aGeq.riskVar_.get(GRB_DoubleAttr_Obj);
    //             }
    //         }
    //     }
    // }

    getProbSuccess(oriRiskNonCost, oriRiskLinCost);
    // printf("getCost: riskCostLin = %f\n", riskCostLin);
    // printf("getCost: oriRiskLinCost = %f\n", oriRiskLinCost);
    // printf("getCost: oriRiskNonCost = %f\n", oriRiskNonCost);

    fCost = energyCost + timeCost + otherCost + flowEnergyCost;
    oriLinCost = fCost + oriRiskLinCost;
    oriNonCost = fCost + oriRiskNonCost;
    // oriLinCost = fCost + thetaCost + oriRiskLinCost;
    // oriNonCost = fCost + recourseCost + oriRiskNonCost;
}

void TeamPlanner::getFinalCost(const std::vector<int>& vehType, const std::vector<double>& vehFlow, const std::vector<std::vector<int>>& vehPath,
                            const std::vector<std::vector<int>>& vehEdge, const std::vector<double>& vehSumEng, const std::vector<double>& vehSumVar,
                            const std::vector<std::vector<int>>& taskTeam, const std::vector<std::vector<double>>& taskTeamDense,
                            double& finalLinCost, double& finalNonCost, double& taskRiskLinCost, double& taskRiskNonCost) {
    double detEngCost = 0.0;
    for (size_t pathId = 0; pathId < vehFlow.size(); pathId++) {
        detEngCost += vehFlow[pathId] * vehSumEng[pathId];
    }

    double flowEngCost = 0.0;
    if (gVar_ != nullptr) {
        for (int i = 0; i < gVarNum_; i++) {
            flowEngCost += gVar_[i].get(GRB_DoubleAttr_X) * gVar_[i].get(GRB_DoubleAttr_Obj);
        }
        // printf("getCost: flowEngCost = %f\n", flowEngCost);
    }

    double yVarCost = 0.0;
    for (int i = 0; i < yVarNum_; i++) {
        yVarCost += yVar_[i].get(GRB_DoubleAttr_X) * yVar_[i].get(GRB_DoubleAttr_Obj);
    }

    double qVarCost = 0.0;
    for (int i = 0; i < qVarNum_; i++) {
        qVarCost += qVar_[i].get(GRB_DoubleAttr_X) * qVar_[i].get(GRB_DoubleAttr_Obj);
    }

    double finalRiskCostNon, finalRiskCostLin;
    getProbSuccess(finalRiskCostNon, finalRiskCostLin, &taskTeamDense);

    double baseCost = detEngCost + flowEngCost + yVarCost + qVarCost;
    taskRiskLinCost   = finalRiskCostLin;
    taskRiskNonCost   = finalRiskCostNon;
    finalLinCost = baseCost + finalRiskCostLin;
    finalNonCost = baseCost + finalRiskCostNon;
}


void TeamPlanner::getTeam(std::vector<std::vector<int>>& taskTeam, const double* yValue) const {
    taskTeam.clear();
    taskTeam.resize(taskNum_);
    // Determine vehicle teams for each task
    for (int i = 0; i < taskNum_; i++) {
        for (int k = 0; k < vehNum_; k++) {
            int yId = sub2yId(k, i);
            double yTemp = 0.0;
            if (yValue == nullptr) {
                yTemp = yVar_[yId].get(GRB_DoubleAttr_X);
            }
            else {
                yTemp = yValue[yId];
            }
            if (yTemp > 0.5) {
                taskTeam[i].push_back(k);
            }
        }
    }
}

void TeamPlanner::getTeamContinuous(std::vector<std::vector<int>>& taskTeam, std::vector<std::vector<double>>& taskTeamDense, const std::vector<int>& vehType, const std::vector<double>& vehFlow, const std::vector<std::vector<int>>& vehPath) const {
    taskTeamDense.clear();
    taskTeamDense.resize(taskNum_);
    for (int i = 0; i < taskNum_; i++) {
        taskTeamDense[i].resize(vehNum_);
        std::fill(taskTeamDense[i].begin(), taskTeamDense[i].end(), 0.0);
    }
    for (size_t pathId = 0; pathId < vehPath.size(); pathId++) {
        for (size_t i = 1; i < vehPath[pathId].size()-1; i++) {
            int veh = vehType[pathId];
            int node = vehPath[pathId][i];
            taskTeamDense[node][veh] += vehFlow[pathId];
        }
    }
    double vehEps = 1e-4;
    taskTeam.clear();
    taskTeam.resize(taskNum_);
    for (int i = 0; i < taskNum_; i++) {
        std::vector<int> aTeam;
        for (int veh = 0; veh < vehNum_; veh++) {
            if (taskTeamDense[i][veh] > vehEps) {
                taskTeam[i].push_back(veh);
            }
        }
    }
}

void TeamPlanner::getPathCover(std::vector<int>& vehType, std::vector<double>& vehFlow, std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue) const {
    for (int k = 0; k < vehNum_; k++) {
        FlowConverter converter;
        int veboseLevel = 0; // 3 for verbose
        converter.setVerboseLevel(veboseLevel);
        converter.initializeEnv();
        // Construct graph
        int startNode = taskNum_ + k;
        int endNode   = taskNum_ + vehNum_ + k;
        for (int nodeId = 0; nodeId < nodeNum_; nodeId++) {
            converter.graph_.addNode(nodeId, 0.0); // Assume nodeId == node // graph_[k].node(nodeId).timeCost_
        }
        for (int edgeId = 0; edgeId < edgeNum_; edgeId++) {
            double flowLB = 0.0;
            int xId = sub2xId(k, edgeId);
            if (xValue == nullptr) {
                flowLB = xVar_[xId].get(GRB_DoubleAttr_X);
            }
            else {
                flowLB = xValue[xId];
            }
            const GraphEdgeParam& an_edge = graph_[k].edge(edgeId);
            int node1 = an_edge.edge_.node1_;
            int node2 = an_edge.edge_.node2_;
            if ( ! (node1 == startNode || node2 == endNode) ) {
                // Avoid cycle in the graph after rounding
                if (flowLB < 0.1) {
                    continue;
                }
            }
            int edgeType = an_edge.edge_.type_;
            double engCost = an_edge.engCost_;
            double flowUB = 100000.0; // engUnc
            bool flagDirected = true;
            converter.graph_.addEdge(node1, node2, edgeType, engCost, flowUB, flowLB, flagDirected);
        }
        // Form the rounding problem

        converter.formRoundProblem(startNode, endNode);
        converter.optimizeRound(param_.solverMaxTime_);

        // converter.graph_.print();
        // converter.printRound();
        // converter.saveRound(roundFile);

        converter.formCoverProblem();
        converter.optimizeCover(param_.solverMaxTime_);
        // converter.printCover();
        // converter.saveCover(coverFile);

        std::vector<std::vector<int>> tempVehPath;
        std::vector<std::vector<int>> tempVehEdge;
        std::vector<double> tempVehSumEng;
        std::vector<double> tempVehSumVar;
        converter.getPath(tempVehPath, tempVehEdge, tempVehSumEng, tempVehSumVar);

        for (size_t pathId = 0; pathId < tempVehPath.size(); pathId++) {
    // std::cout << "\npathId = " << pathId << "\n";
            double aEng = 0.0;
            double aEngVar = 0.0;
            const std::vector<int>& aPathNode = tempVehPath[pathId];
            std::vector<int> aPathEdge(tempVehEdge[pathId].size());

            for (int insideId = 0; insideId < tempVehEdge[pathId].size(); insideId++) {
                const GraphEdgeParam& insideEdge = converter.graph_.edge(tempVehEdge[pathId][insideId]);
    // std::cout << "node = " << aPathNode[insideId] << " , " << aPathNode[insideId+1] << "\n";
    // std::cout << "insideEdge.edge_ = " << insideEdge.edge_.node1_ << "," << insideEdge.edge_.node2_ << "," << insideEdge.edge_.type_ << ", eng: " << insideEdge.engCost_ << "\n";
                int edgeId = graph_[k].edge2id(insideEdge.edge_.node1_, insideEdge.edge_.node2_, insideEdge.edge_.type_);
    // std::cout << "edgeId = " << edgeId << "\n";
                aPathEdge[insideId] = edgeId;
                const GraphEdgeParam& aEdge = graph_[k].edge(edgeId);
                aEng += aEdge.engCost_;
                aEngVar += aEdge.engUnc_;
    // std::cout << "aEdge.engCost_ = " << aEdge.engCost_ << "\n";
            }

            vehType.push_back(k);
            vehFlow.push_back(1.0);
            vehPath.push_back(aPathNode);
            vehEdge.push_back(aPathEdge);
            vehSumEng.push_back(tempVehSumEng[pathId]);
            vehSumVar.push_back(aEngVar);
        }
    }
}

void TeamPlanner::getPathContinuous(std::vector<int>& vehType, std::vector<double>& vehFlow, std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue) const {
    double xEps = 1e-2;
    double* xResidue = new double[xVarNum_];
    for (int xId = 0; xId < xVarNum_; xId++) {
        if (xValue == nullptr) {
            xResidue[xId] = xVar_[xId].get(GRB_DoubleAttr_X);
        }
        else {
            xResidue[xId] = xValue[xId];
        }
    }
    for (int k = 0; k < vehNum_; ) {
        int xIdBegin = edgeOffset_[k];
        int xIdEnd;
        if (k < vehNum_-1) {
            xIdEnd = edgeOffset_[k+1];
        }
        else {
            xIdEnd = xVarNum_;
        }

        // int xIdMin = std::min_element(xResidue+xIdBegin, xResidue+xIdEnd) - xResidue;
        // double xMin = xResidue[xIdMin];
        int xIdMin = -1;
        double xMin = 1000000.0;
        for (int xId = xIdBegin; xId < xIdEnd; xId++) {
            if (xResidue[xId] < xEps) {
                continue;
            }
            if (xResidue[xId] < xMin) {
                xIdMin = xId;
                xMin = xResidue[xId];
            }
        }
        if (xIdMin < 0) {
            k++;
            continue;
        }
        xResidue[xIdMin] = 0.0;
        std::vector<int> aPathNode;
        std::vector<int> aPathNodeBefore;
        std::vector<int> aPathNodeAfter;
        std::vector<int> aPathEdge;
        std::vector<int> aPathEdgeBefore;
        std::vector<int> aPathEdgeAfter;
        double aEng = 0.0;
        double aEngVar = 0.0;
        int minEdge = xIdMin - xIdBegin;
        int minNode1 = graph_[k].edge(minEdge).edge_.node1_;
        int minNode2 = graph_[k].edge(minEdge).edge_.node2_;

        aEng += xVar_[xIdMin].get(GRB_DoubleAttr_Obj);
        aEngVar += graph_[k].edge(minEdge).engUnc_;

        aPathNodeAfter.push_back(minNode2);
        for (int i = minNode2; ;) {
            int travelEdgeId = -1;
            for (int edgeId : graph_[k].node(i).outEdgeIds_) {
                int xId = sub2xId(k, edgeId);
                double x = 0.0;
                // Based on whether getPath is called in a callback function
                // if (xResidue == nullptr) {
                //     x = xVar_[xId].get(GRB_DoubleAttr_X);
                // }
                // else {
                //     x = xResidue[xId];
                // }
                if (xResidue[xId] >= xMin) {
                    travelEdgeId = edgeId;
                    aEng += xVar_[xId].get(GRB_DoubleAttr_Obj);
                    aEngVar += graph_[k].edge(edgeId).engUnc_;
                    xResidue[xId] -= xMin;
                    break;
                }
            }
            if (travelEdgeId >= 0) {
                int j = graph_[k].edge(travelEdgeId).edge_.node2_;
                aPathNodeAfter.push_back(j);
                aPathEdgeAfter.push_back(travelEdgeId);
                i = j;
            }
            else {
                break;
            }
        }

        aPathNodeBefore.push_back(minNode1);
        for (int i = minNode1; ;) {
            int travelEdgeId = -1;
            for (int edgeId : graph_[k].node(i).inEdgeIds_) {
                int xId = sub2xId(k, edgeId);
                double x = 0.0;
                // Based on whether getPath is called in a callback function
                // if (xResidue == nullptr) {
                //     x = xVar_[xId].get(GRB_DoubleAttr_X);
                // }
                // else {
                //     x = xResidue[xId];
                // }
                if (xResidue[xId] >= xMin) {
                    travelEdgeId = edgeId;
                    aEng += xVar_[xId].get(GRB_DoubleAttr_Obj);
                    aEngVar += graph_[k].edge(edgeId).engUnc_;
                    xResidue[xId] -= xMin;
                    break;
                }
            }
            if (travelEdgeId >= 0) {
                int j = graph_[k].edge(travelEdgeId).edge_.node1_;
                aPathNodeBefore.push_back(j);
                aPathEdgeBefore.push_back(travelEdgeId);
                i = j;
            }
            else {
                break;
            }
        }

        for (int i = static_cast<int>(aPathNodeBefore.size())-1; i >= 0; i--) {
            aPathNode.push_back(aPathNodeBefore[i]);
        }
        for (int i = 0; i < static_cast<int>(aPathNodeAfter.size()); i++) {
            aPathNode.push_back(aPathNodeAfter[i]);
        }
        for (int i = static_cast<int>(aPathEdgeBefore.size())-1; i >= 0; i--) {
            aPathEdge.push_back(aPathEdgeBefore[i]);
        }
        aPathEdge.push_back(minEdge);
        for (int i = 0; i < static_cast<int>(aPathEdgeAfter.size()); i++) {
            aPathEdge.push_back(aPathEdgeAfter[i]);
        }

        vehType.push_back(k);
        vehFlow.push_back(xMin);
        vehPath.push_back(aPathNode);
        vehEdge.push_back(aPathEdge);
        vehSumEng.push_back(aEng);
        vehSumVar.push_back(aEngVar);

    }
    delete[] xResidue;
}

void TeamPlanner::getPath(std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue) const {
    vehPath.clear();
    vehPath.resize(vehNum_);
    vehEdge.clear();
    vehEdge.resize(vehNum_);
    vehSumEng.clear();
    vehSumEng.resize(vehNum_);
    vehSumVar.clear();
    vehSumVar.resize(vehNum_);
    // Determine path for each vehicle
    for (int k = 0; k < vehNum_; k++) {
        vehPath[k].clear();
        vehEdge[k].clear();
        vehSumEng[k] = 0.0;
        vehSumVar[k] = 0.0;
        for (int i = taskNum_ + k; ;) {
            int travelEdgeId = -1;
            for (int edgeId : graph_[k].node(i).outEdgeIds_) {
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
                    travelEdgeId = edgeId;
                    vehSumEng[k] += xVar_[xId].get(GRB_DoubleAttr_Obj);
                    vehSumVar[k] += graph_[k].edge(edgeId).engUnc_;
                    break;
                }
            }
            if (travelEdgeId >= 0) {
                if (i == taskNum_ + k) {
                    vehPath[k].push_back(i);
                }
                int j = graph_[k].edge(travelEdgeId).edge_.node2_;
                vehPath[k].push_back(j);
                vehEdge[k].push_back(travelEdgeId);
                i = j;
            }
            else {
                break;
            }
        }
    }
}

void TeamPlanner::printSolution(std::vector<std::vector<int>>* vehPathOut, std::vector<std::vector<int>>* taskTeamOut, std::vector<std::vector<double>>* taskTeamDenseOut) {
    // for (int xId = 0; xId < xVarNum_; xId++) {
    //     if (xVar_[xId].get(GRB_DoubleAttr_X) > 0.5)
    //         std::cout << xVar_[xId].get(GRB_StringAttr_VarName) << " = " << xVar_[xId].get(GRB_DoubleAttr_X) << "\n";
    // }
    // for (int yId = 0; yId < yVarNum_; yId++) {
    //     if (yVar_[yId].get(GRB_DoubleAttr_X) > 0.5)
    //         std::cout << yVar_[yId].get(GRB_StringAttr_VarName) << " = " << yVar_[yId].get(GRB_DoubleAttr_X) << "\n";
    // }
    // for (int zId = 0; zId < zVarNum_; zId++) {
    //     std::cout << zVar_[zId].get(GRB_StringAttr_VarName) << " = " << zVar_[zId].get(GRB_DoubleAttr_X) << "\n";
    // }
    if (!flagOptimized_) {
        std::cout << "optStatus: " << model_->get(GRB_IntAttr_Status) << "\n";
        return;
    }
    // for (int qId = 0; qId < qVarNum_; qId++) {
    //     if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
    //         std::cout << qVar_[qId].get(GRB_StringAttr_VarName) << " = " << bestSolution_.qVar_[qId];
    //     }
    //     else {
    //         std::cout << qVar_[qId].get(GRB_StringAttr_VarName) << " = " << qVar_[qId].get(GRB_DoubleAttr_X) << "\n";
    //     }
    // }
    std::vector<int> vehType;
    std::vector<double> vehFlow;
    std::vector<std::vector<int>> vehPath;
    std::vector<std::vector<int>> vehEdge;
    std::vector<std::vector<int>> taskTeam;
    std::vector<std::vector<double>> taskTeamDense;
    std::vector<double> vehSumEng;
    std::vector<double> vehSumVar;
    std::vector<double> vehRecourseCost;
    bool flagOffset = true;
    double finalLinCost = -1.0;
    double finalNonCost = -1.0;
    double oriLinCost = -1.0;
    double oriNonCost = -1.0;
    double finalRiskLinCost = -1.0;
    double finalRiskNonCost = -1.0;
    double oriRiskLinCost = -1.0;
    double oriRiskNonCost = -1.0;

    double startPathTime = GetTime(); // Mark the start time
    double energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost;
    if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
        getPath(vehPath, vehEdge, vehSumEng, vehSumVar, bestSolution_.xVar_);
        getTeam(taskTeam, bestSolution_.yVar_);
        getRecourseCost(vehRecourseCost, bestSolution_.xVar_);
        getCost(vehRecourseCost, energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost);
    }
    else if (flagContinuous_) {
        if (param_.flagFlowCover_) {
            getPathCover(vehType, vehFlow, vehPath, vehEdge, vehSumEng, vehSumVar);
        }
        else {
            getPathContinuous(vehType, vehFlow, vehPath, vehEdge, vehSumEng, vehSumVar);
        }
        // getTeam(taskTeam); // Still correct
        getTeamContinuous(taskTeam, taskTeamDense, vehType, vehFlow, vehPath);
        getRecourseCost(vehRecourseCost); // Problematic
        getCost(vehRecourseCost, energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost);
        getFinalCost(vehType, vehFlow, vehPath, vehEdge, vehSumEng, vehSumVar, taskTeam, taskTeamDense, finalLinCost, finalNonCost, finalRiskLinCost, finalRiskNonCost);
    }
    else {
        getPath(vehPath, vehEdge, vehSumEng, vehSumVar);
        getTeam(taskTeam);
        getRecourseCost(vehRecourseCost);
        getCost(vehRecourseCost, energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost);
        finalLinCost = oriLinCost;
        finalNonCost = oriNonCost;
        finalRiskLinCost = oriRiskLinCost;
        finalRiskNonCost = oriRiskNonCost;
    }
    double pathTime = GetTime() - startPathTime; // Mark the start time

    if (vehPathOut != nullptr || taskTeamOut != nullptr) {
        if (vehPathOut != nullptr) {
            *vehPathOut = vehPath;
        }
        if (taskTeamOut != nullptr) {
            *taskTeamOut = taskTeam;
        }
        if (taskTeamDenseOut != nullptr) {
            *taskTeamDenseOut = taskTeamDense;
        }
        return;
    }

    std::cout << "Approximation CVaR: " << finalRiskLinCost << "Task CVaR, \t : " << finalRiskNonCost << "\n";
    for (int i = 0; i < taskNum_; i++) {
        std::cout << toString(TEAMPLANNER_TASK, i) << ": " << taskVar_[i].probSuccess_ << "\n";
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                const TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                if (flagTaskVarAdded_ && !param_.flagFlowCover_ ) {
                    std::cout << aGeq.riskVar_.get(GRB_StringAttr_VarName) << " = " << aGeq.riskVar_.get(GRB_DoubleAttr_X) << "\t\t";
                    std::cout << aGeq.helpVar_.get(GRB_StringAttr_VarName) << " = " << aGeq.helpVar_.get(GRB_DoubleAttr_X) << "\t\t";
                }
                std::cout << "risk[" << toString(TEAMPLANNER_TASK, i) << "," << andId << "," << orId << "] = " << aGeq.riskCost_ << "\t\t";
                std::cout << "P() = " << aGeq.probSuccess_ << "\n";
            }
        }
    }

    // std::cout << "\n\n";
    // double riskCostNon, riskCostLin;
    // getProbSuccess(riskCostNon, riskCostLin);
    // std::cout << "Approximation CVaR: " << riskCostLin << "Task CVaR, \t : " << riskCostNon << "\n";
    // for (int i = 0; i < taskNum_; i++) {
    //     std::cout << toString(TEAMPLANNER_TASK, i) << ": " << taskVar_[i].probSuccess_ << "\n";
    //     for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
    //         for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
    //             const TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
    //             if (flagTaskVarAdded_) {
    //                 std::cout << aGeq.riskVar_.get(GRB_StringAttr_VarName) << " = " << aGeq.riskVar_.get(GRB_DoubleAttr_X) << "\t\t";
    //                 std::cout << aGeq.helpVar_.get(GRB_StringAttr_VarName) << " = " << aGeq.helpVar_.get(GRB_DoubleAttr_X) << "\t\t";
    //             }
    //             std::cout << "risk[" << toString(TEAMPLANNER_TASK, i) << "," << andId << "," << orId << "] = " << aGeq.riskCost_ << "\t\t";
    //             std::cout << "P() = " << aGeq.probSuccess_ << "\n";
    //         }
    //     }
    // }

    std::cout << "\nVehicle paths:\n";
    int iterNum = vehNum_;
    if (flagContinuous_) {
        iterNum = static_cast<int>(vehType.size());
    }
    for (int k = 0; k < iterNum; k++) {
        int veh = k;
        if (vehPath[k].size() == 0) {
            continue;
        }
        if (flagContinuous_) {
            veh = vehType[k];
        }
        std::cout << toString(TEAMPLANNER_VEHCTYPE, veh, flagOffset);
        if (flagContinuous_) {
            if (fabs(vehFlow[k] - 1.0) > 0.01) {
                std::cout << "-" << vehFlow[k];
            }
        }
        std::cout << ":\t";
        for (int pathId = 0; pathId < static_cast<int>(vehPath[k].size()); pathId++) {
            if (pathId > 0) {
                std::cout << " --> ";
            }
            std::cout << toString(TEAMPLANNER_NODE, vehPath[k][pathId], flagOffset);
        }
        std::cout << "\t\teng:\t" << vehSumEng[k] << " < " << vehSumEng[k] + normalCDFInverse(param_.CcpBeta_) * sqrt(vehSumVar[k]) << " < "
            << param_.vehParam_[veh].engCap_ << " , " << vehRecourseCost[veh];
        if (param_.flagSolver_ == TEAMPLANNER_SPR) {
            std::cout << " == " << thetaVar_[k].get(GRB_DoubleAttr_X) << " == " << thetaVar_[k].get(GRB_StringAttr_VarName) << "\n";
        }
        else if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            std::cout << " == " << bestSolution_.thetaVar_[k] << " == " << thetaVar_[k].get(GRB_StringAttr_VarName) << "\n";
        }
        std::cout << "\n";
    }
    std::cout << "\nTask teams:\n";
    for (int i = 0; i < taskNum_; i++) {
        std::cout << toString(TEAMPLANNER_TASK, i, flagOffset) << ":\t";
        for (int k = 0; k < static_cast<int>(taskTeam[i].size()); k++) {
            if (k > 0) {
                std::cout << ", ";
            }
            std::cout << toString(TEAMPLANNER_VEHCTYPE, taskTeam[i][k], flagOffset);
            if (flagContinuous_) {
                // int yId = sub2yId(taskTeam[i][k], i);
                // double yTemp = yVar_[yId].get(GRB_DoubleAttr_X);
                int veh = taskTeam[i][k];
                double yTemp = taskTeamDense[i][veh];
                if (fabs(yTemp - 1.0) > 0.01) {
                    std::cout << "-" << yTemp;
                }
            }
        }
        std::cout << "\n";
    }

    double lowerBound = model_->get(GRB_DoubleAttr_ObjBound);
    double oriLinGap = (oriLinCost - lowerBound) / oriLinCost;
    double oriNonGap = (oriNonCost - lowerBound) / oriNonCost;
    double finalLinGap = (finalLinCost - lowerBound) / finalLinCost;
    double finalNonGap = (finalNonCost - lowerBound) / finalNonCost;
    std::cout << "optStatus: " << model_->get(GRB_IntAttr_Status) << "\n";
    std::cout << "objVal: " << model_->get(GRB_DoubleAttr_ObjVal) << "\n";
    std::cout << "objBound: " << lowerBound << "\n";
    std::cout << "MIPGap: " << model_->get(GRB_DoubleAttr_MIPGap) << "\n";
    std::cout << "numVars: " << model_->get(GRB_IntAttr_NumVars) << "\n";
    std::cout << "numConstrs: " << model_->get(GRB_IntAttr_NumConstrs) << "\n";
    std::cout << "solCount: " << model_->get(GRB_IntAttr_SolCount) << "\n";
    std::cout << "nodeCount: " << model_->get(GRB_DoubleAttr_NodeCount) << "\n";
    std::cout << "iterCount: " << model_->get(GRB_DoubleAttr_IterCount) << "\n";
    std::cout << "constraintNum_: " << constraintNum_ << "\n";
    std::cout << "cutNum_: " << cutNum_ << "\n";
    std::cout << "solverTime_: " << solverTime_ << "\n";
    std::cout << "runTime: " << model_->get(GRB_DoubleAttr_Runtime) << "\n";
    std::cout << "pathTime: " << pathTime << "\n";
    std::cout << "plannerTime: " << solverTime_ + pathTime << "\n";
    std::cout << "oriLinCost: " << oriLinCost << "\n";
    std::cout << "oriNonCost: " << oriNonCost << "\n";
    std::cout << "oriLinGap: " << oriLinGap << "\n";
    std::cout << "oriNonGap: " << oriNonGap << "\n";
    std::cout << "oriRiskLinCost: " << oriRiskLinCost << "\n";
    std::cout << "oriRiskNonCost: " << oriRiskNonCost << "\n";
    std::cout << "finalLinCost: " << finalLinCost << "\n";
    std::cout << "finalNonCost: " << finalNonCost << "\n";
    std::cout << "finalLinGap: " << finalLinGap << "\n";
    std::cout << "finalNonGap: " << finalNonGap << "\n";
    std::cout << "finalRiskLinCost: " << finalRiskLinCost << "\n";
    std::cout << "finalRiskNonCost: " << finalRiskNonCost << "\n";

    std::cout << "energyCost = " << energyCost << "\n"; 
    std::cout << "timeCost = " << timeCost << "\n"; 
    std::cout << "otherCost = " << otherCost << "\n"; 
    std::cout << "fCost = " << fCost << "\n"; 
    std::cout << "thetaCost = " << thetaCost << "\n"; 
    std::cout << "recourseCost = " << recourseCost << "\n"; 
    std::cout << "flagSuccess: " << flagSuccess_ << "\n";

}

void TeamPlanner::saveSamples(const std::string& saveFileName) const {
    std::ofstream logfile(saveFileName);
    for (int i = 0; i < taskNum_; i++) {
        logfile << "task" << i << ":\n";
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            logfile << "  and" << andId << ":\n";
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                logfile << "    or" << orId << ": [";
                const TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                for (size_t s = 0; s < aGeq.capSample_.size(); s++) {
                    if (s > 0) {
                        logfile << ", ";
                    }
                    logfile << std::fixed << std::setprecision(8) << aGeq.capSample_[s];
                }
                logfile << "]\n";
            }
        }
    }
    for (int k = 0; k < vehNum_; k++) {
        logfile << "vehicle" << k << ":\n";
        for (size_t a = 0; a < vehVar_[k].capSample_.size(); a++) {
            logfile << "  cap" << a << ": [";
            for (size_t s = 0; s < vehVar_[k].capSample_[a].size(); s++) {
                if (s > 0) {
                    logfile << ", ";
                }
                logfile << std::fixed << std::setprecision(8) << vehVar_[k].capSample_[a][s];
            }
            logfile << "]\n";
        }
    }
}

template<class T>
std::string TeamPlanner::toString(const std::vector<T>& data) const {
    std::string str = "[";
    for (size_t i = 0; i < data.size(); i++) {
        if (i > 0) {
            str += ", ";
        }
        str += std::to_string(data[i]);
    }
    str += "]";
    return str;
}

void TeamPlanner::saveSolution(const std::string& saveFileName, const std::string& paramFileName) {
    if (!flagOptimized_) {
        std::ofstream logfile(saveFileName);
        logfile << "result:\n";
        logfile << "  flagSuccess: " << false << "\n";
        logfile << "  flagOptimized: " << false << "\n";
        logfile << "  optStatus: " << model_->get(GRB_IntAttr_Status) << "\n";
        YAML::Node yamlParam = YAML::LoadFile(paramFileName);
        logfile << yamlParam;
        logfile << "\n";
        logfile.close();
        return;
    }
    std::ofstream logfile(saveFileName);
    // std::ofstream logfile;
    // logfile.open(saveFileName, std::ios_base::app);
    std::vector<int> vehType;
    std::vector<double> vehFlow;
    std::vector<std::vector<int>> vehPath;
    std::vector<std::vector<int>> vehEdge;
    std::vector<std::vector<int>> taskTeam;
    std::vector<std::vector<double>> taskTeamDense;
    std::vector<double> vehSumEng;
    std::vector<double> vehSumVar;
    std::vector<double> vehRecourseCost;
    double finalLinCost = -1.0;
    double finalNonCost = -1.0;
    double oriLinCost = -1.0;
    double oriNonCost = -1.0;
    double finalRiskLinCost = -1.0;
    double finalRiskNonCost = -1.0;
    double oriRiskLinCost = -1.0;
    double oriRiskNonCost = -1.0;

    double startPathTime = GetTime(); // Mark the start time
    bool flagOffset = true;
    bool flagNumberOnly = true;
    double energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost;
    if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
        getPath(vehPath, vehEdge, vehSumEng, vehSumVar, bestSolution_.xVar_);
        getTeam(taskTeam, bestSolution_.yVar_);
        getRecourseCost(vehRecourseCost, bestSolution_.xVar_);
        getCost(vehRecourseCost, energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost);
    }
    else if (flagContinuous_) {
        if (param_.flagFlowCover_) {
            getPathCover(vehType, vehFlow, vehPath, vehEdge, vehSumEng, vehSumVar);
        }
        else {
            getPathContinuous(vehType, vehFlow, vehPath, vehEdge, vehSumEng, vehSumVar);
        }
        // getTeam(taskTeam); // Still correct
        getTeamContinuous(taskTeam, taskTeamDense, vehType, vehFlow, vehPath);
        getRecourseCost(vehRecourseCost); // Problematic
        getCost(vehRecourseCost, energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost);
        getFinalCost(vehType, vehFlow, vehPath, vehEdge, vehSumEng, vehSumVar, taskTeam, taskTeamDense, finalLinCost, finalNonCost, finalRiskLinCost, finalRiskNonCost);
        // getProbSuccess(riskCostNon, riskCostLin, &taskTeamDense);
    }
    else {
        getPath(vehPath, vehEdge, vehSumEng, vehSumVar);
        getTeam(taskTeam);
        getRecourseCost(vehRecourseCost);
        getCost(vehRecourseCost, energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost);
        // getProbSuccess(riskCostNon, riskCostLin);
        finalLinCost = oriLinCost;
        finalNonCost = oriNonCost;
        finalRiskLinCost = oriRiskLinCost;
        finalRiskNonCost = oriRiskNonCost;
    }
    double pathTime = GetTime() - startPathTime; // Mark the start time

    // Calculate bound and optimality gaps
    double lowerBound = model_->get(GRB_DoubleAttr_ObjBound);
    double oriLinGap = (oriLinCost - lowerBound) / oriLinCost;
    double oriNonGap = (oriNonCost - lowerBound) / oriNonCost;
    double finalLinGap = (finalLinCost - lowerBound) / finalLinCost;
    double finalNonGap = (finalNonCost - lowerBound) / finalNonCost;

    logfile << "result:\n";
    logfile << std::fixed << std::setprecision(6) << "  energyCost: " << energyCost << "\n";
    logfile << "  timeCost: " << timeCost << "\n";
    logfile << "  otherCost: " << otherCost << "\n";
    logfile << "  fCost: " << fCost << "\n";
    logfile << "  thetaCost: " << thetaCost << "\n"; 
    logfile << "  recourseCost: " << recourseCost << "\n";
    logfile << "  flagSuccess: " << flagSuccess_ << "\n";
    logfile << "  optStatus: " << model_->get(GRB_IntAttr_Status) << "\n";

    logfile << "  objVal: " << model_->get(GRB_DoubleAttr_ObjVal) << "\n";
    logfile << "  objBound: " << lowerBound << "\n";
    logfile << "  MIPGap: " << model_->get(GRB_DoubleAttr_MIPGap) << "\n";
    logfile << "  varsNum: " << model_->get(GRB_IntAttr_NumVars) << "\n";
    logfile << "  constraintNum: " << constraintNum_ << "\n";
    logfile << "  cutNum: " << cutNum_ << "\n";
    logfile << "  solCount: " << model_->get(GRB_IntAttr_SolCount) << "\n";
    logfile << "  nodeCount: " << model_->get(GRB_DoubleAttr_NodeCount) << "\n";
    logfile << "  iterCount: " << model_->get(GRB_DoubleAttr_IterCount) << "\n";
    logfile << "  solverTime: " << solverTime_ << "\n";
    logfile << "  lastSolverTime: " << model_->get(GRB_DoubleAttr_Runtime) << "\n";
    logfile << "  pathTime: " << pathTime << "\n";
    logfile << "  plannerTime: " << solverTime_ + pathTime << "\n";

    logfile << "  oriLinCost: " << oriLinCost << "\n";
    logfile << "  oriNonCost: " << oriNonCost << "\n";
    logfile << "  oriLinGap: " << oriLinGap << "\n";
    logfile << "  oriNonGap: " << oriNonGap << "\n";
    logfile << "  oriRiskLinCost: " << oriRiskLinCost << "\n";
    logfile << "  oriRiskNonCost: " << oriRiskNonCost << "\n";
    logfile << "  finalLinCost: " << finalLinCost << "\n";
    logfile << "  finalNonCost: " << finalNonCost << "\n";
    logfile << "  finalLinGap: " << finalLinGap << "\n";
    logfile << "  finalNonGap: " << finalNonGap << "\n";
    logfile << "  finalRiskLinCost: " << finalRiskLinCost << "\n";
    logfile << "  finalRiskNonCost: " << finalRiskNonCost << "\n";

    logfile << "  taskCvarLin: " << finalRiskLinCost << "\n"; // Deprecated
    logfile << "  taskCvarNon: " << finalRiskNonCost << "\n"; // Deprecated

    logfile << "team:\n";
    for (int i = 0; i < taskNum_; i++) {
        logfile << "  " << toString(TEAMPLANNER_TASK, i, flagOffset) << ":\n";
        std::vector<int> vehTypeCount(vehTypeNum_);
        std::fill(vehTypeCount.begin(), vehTypeCount.end(), 0);
        logfile << "    id: [";
        for (int k = 0; k < static_cast<int>(taskTeam[i].size()); k++) {
            vehTypeCount[taskTeam[i][k] % vehTypeNum_]++;
            if (k > 0) {
                logfile << ", ";
            }
            logfile << toString(TEAMPLANNER_VEHC, taskTeam[i][k], flagOffset, flagNumberOnly);
        }
        logfile << "]\n";
        logfile << "    count: [";
        for (int k = 0; k < vehTypeNum_; k++) {
            if (k > 0) {
                logfile << ", ";
            }
            if (flagContinuous_) {
                // int yId = sub2yId(k, i);
                // logfile << yVar_[yId].get(GRB_DoubleAttr_X);
                logfile << taskTeamDense[i][k];
            }
            else {
                logfile << vehTypeCount[k];
            }
        }
        logfile << "]\n";

        std::vector<int> tempCapId;
        // std::vector<double> tempCapReq;
        // std::vector<double> tempCapNum;
        std::vector<double> tempNonRisk;
        std::vector<double> tempRiskVar;
        std::vector<double> tempHelpVar;
        std::vector<double> tempProbSuccess;
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                const TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                tempCapId.push_back(aGeq.capId_);
                if (flagTaskVarAdded_) {
                    tempRiskVar.push_back(aGeq.riskVar_.get(GRB_DoubleAttr_X));
                    tempHelpVar.push_back(aGeq.helpVar_.get(GRB_DoubleAttr_X));
                }
                tempProbSuccess.push_back(aGeq.probSuccess_);
                tempNonRisk.push_back(aGeq.riskCost_);
            }
        }
        logfile << "    taskProbSuccess: " << taskVar_[i].probSuccess_ << "\n";
        logfile << "    taskNonRisk: " << taskVar_[i].riskCost_ << "\n";
        logfile << "    capId: " << toString(tempCapId) << "\n";
        logfile << "    taskCapNonRisk: " << toString(tempNonRisk) << "\n";
        logfile << "    taskCapRiskVar: " << toString(tempRiskVar) << "\n";
        logfile << "    taskCapHelpVar: " << toString(tempHelpVar) << "\n";
        logfile << "    taskCapProbSuccess: " << toString(tempProbSuccess) << "\n";
    }

    int iterNum = vehNum_;
    if (flagContinuous_) {
        iterNum = static_cast<int>(vehType.size());
    }
    logfile << "vehicle:\n";
    for (int k = 0, kk = 0; k < iterNum; k++) {
        if (vehPath[k].size() == 0) {
            continue;
        }
        int veh = k;
        double aFlow = 1.0;
        if (flagContinuous_) {
            veh = vehType[k];
            aFlow = vehFlow[k];
        }
        kk++;
        logfile << "  vv" << kk << ":\n";
        logfile << "    id: " << toString(TEAMPLANNER_VEHC, veh, flagOffset, flagNumberOnly) << "\n";
        logfile << "    type: " << toString(TEAMPLANNER_VEHCTYPE, veh, flagOffset, flagNumberOnly) << "\n";
        logfile << "    flow: " << aFlow << "\n";
        logfile << "    vehEngMu: " << vehSumEng[k] << "\n";
        logfile << "    vehEngVar: " << vehSumVar[k] << "\n";
        logfile << "    vehEngMuVar: " << vehSumEng[k] + normalCDFInverse(param_.CcpBeta_) * sqrt(vehSumVar[k]) << "\n";
        logfile << "    vehEngCap: " << param_.vehParam_[veh].engCap_ << "\n";
        logfile << "    vehRecourseCost:  " << vehRecourseCost[veh] << "\n";
        logfile << "    vehThetaCost:  ";
        if (thetaVar_ !=nullptr) {
            if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
                logfile << bestSolution_.thetaVar_[veh] << "\n";
            }
            else {
                logfile << thetaVar_[veh].get(GRB_DoubleAttr_X) << "\n";
            }
        }
        else {
            logfile << -1 << "\n";
        }
        logfile << "    node: [";
        for (int pathId = 0; pathId < static_cast<int>(vehPath[k].size()); pathId++) {
            if (pathId > 0) {
                logfile << ", ";
            }
            logfile << toString(TEAMPLANNER_NODE, vehPath[k][pathId], flagOffset, flagNumberOnly);
        }
        logfile << "]\n";
        logfile << "    nodeTime: [";
        for (int pathId = 0; pathId < static_cast<int>(vehPath[k].size()); pathId++) {
            if (pathId > 0) {
                logfile << ", ";
            }
            int nodeId = vehPath[k][pathId];
            if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
                logfile << bestSolution_.qVar_[nodeId];
            }
            else {
                logfile << qVar_[nodeId].get(GRB_DoubleAttr_X);
            }
            toString(TEAMPLANNER_NODE, vehPath[k][pathId], flagOffset, flagNumberOnly);
        }
        logfile << "]\n";
        logfile << "    edgeEngMu: [";
        for (int pathId = 0; pathId < static_cast<int>(vehEdge[k].size()); pathId++) {
            if (pathId > 0) {
                logfile << ", ";
            }            
            logfile << graph_[veh].edge(vehEdge[k][pathId]).engCost_;
        }
        logfile << "]\n";
        logfile << "    edgeEngVar: [";
        for (int pathId = 0; pathId < static_cast<int>(vehEdge[k].size()); pathId++) {
            if (pathId > 0) {
                logfile << ", ";
            }            
            logfile << graph_[veh].edge(vehEdge[k][pathId]).engUnc_;
        }
        logfile << "]\n";
        logfile << "    edgeTime: [";
        for (int pathId = 0; pathId < static_cast<int>(vehEdge[k].size()); pathId++) {
            if (pathId > 0) {
                logfile << ", ";
            }            
            logfile << graph_[veh].edge(vehEdge[k][pathId]).timeCost_;
        }
        logfile << "]\n";
    }

    YAML::Node yamlParam = YAML::LoadFile(paramFileName);
    logfile << yamlParam;
    logfile << "\n";

    logfile << "xVar: [";
    for (int i = 0; i < xVarNum_; i++) {
        if (i > 0) {
            logfile << ", ";
        }
        if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            logfile << bestSolution_.xVar_[i];
        }
        else {
            logfile << xVar_[i].get(GRB_DoubleAttr_X);
        }
    }
    logfile << "]\n";
    logfile << "yVar: [";
    for (int i = 0; i < yVarNum_; i++) {
        if (i > 0) {
            logfile << ", ";
        }
        if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            logfile << bestSolution_.yVar_;
        }
        else {
            logfile << yVar_[i].get(GRB_DoubleAttr_X);
        }
    }
    logfile << "]\n";
    logfile << "zVar: [";
    for (int i = 0; i < zVarNum_; i++) {
        if (i > 0) {
            logfile << ", ";
        }
        if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            logfile << bestSolution_.zVar_[i];
        }
        else {
            logfile << zVar_[i].get(GRB_DoubleAttr_X);
        }
    }
    logfile << "]\n";
    logfile << "qVar: [";
    for (int i = 0; i < qVarNum_; i++) {
        if (i > 0) {
            logfile << ", ";
        }
        if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            logfile << bestSolution_.qVar_[i];
        }
        else {
            logfile << qVar_[i].get(GRB_DoubleAttr_X);
        }
    }
    logfile << "]\n";
    logfile << "alphaVar: [";
    for (size_t i = 0; i < alphaVar_.size(); i++) {
        if (i > 0) {
            logfile << ", ";
        }
        if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            logfile << bestSolution_.alphaVar_[i];
        }
        else {
            logfile << alphaVar_[i].get(GRB_DoubleAttr_X);
        }
    }
    logfile << "]\n";
    logfile << "wVar: [";
    for (size_t i = 0; i < wVar_.size(); i++) {
        if (i > 0) {
            logfile << ", ";
        }
        if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            logfile << bestSolution_.wVar_;
        }
        else {
            logfile << wVar_[i].get(GRB_DoubleAttr_X);
        }
    }
    logfile << "]\n";
    logfile << "thetaVar: [";
    for (int i = 0; i < vehNum_; i++) {
        if (i > 0) {
            logfile << ", ";
        }
        if (thetaVar_ != nullptr) {
            if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
                logfile << bestSolution_.thetaVar_[i];
            }
            else {
                logfile << thetaVar_[i].get(GRB_DoubleAttr_X);
            }
        }
        else {
            logfile << 0;
        }
    }
    logfile << "]\n";

    if (xrVar_ != nullptr) {
        logfile << "xrVar: [";
        for (int i = 0; i < xVarNum_; i++) {
            if (i > 0) {
                logfile << ", ";
            }
            logfile << xrVar_[i].get(GRB_DoubleAttr_X);
        }
        logfile << "]\n";
    }
    if (yrVar_ != nullptr) {
        logfile << "yrVar: [";
        for (int i = 0; i < yVarNum_; i++) {
            if (i > 0) {
                logfile << ", ";
            }
            logfile << yrVar_[i].get(GRB_DoubleAttr_X);
        }
        logfile << "]\n";
    }
    if (gVar_ != nullptr) {
        logfile << "gVar: [";
        for (int i = 0; i < gVarNum_; i++) {
            if (i > 0) {
                logfile << ", ";
            }
            logfile << gVar_[i].get(GRB_DoubleAttr_X);
        }
        logfile << "]\n";
    }

    logfile.close();
}

