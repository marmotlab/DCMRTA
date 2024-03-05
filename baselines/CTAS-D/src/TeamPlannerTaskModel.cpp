#include <TeamPlanner.hpp>

void TeamPlanner::formTaskRiskCost() {
    double betaRelated = normalCDFInverse(param_.taskBeta_);
    for (int i = 0; i < taskNum_; i++) {
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                std::string commonName  = toString(TEAMPLANNER_TASK, i)     + ","
                                        + toString(TEAMPLANNER_AND , andId) + ","
                                        + toString(TEAMPLANNER_OR  , orId);
                if (param_.capType_[aGeq.capId_] == 1) {
                    // Non-cumulative capability
                    // GRBLinExpr constraint = - aGeq.riskVar_;
                    double tempTaskCapMu = param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                    double tempTaskCapVar = param_.taskParam_[i].reqFcn_[andId][orId].capVar_;
                    for (int k = 0; k < vehNum_; k++) {
                        double tempVehCapMu = param_.vehParam_[k].capVector_[aGeq.capId_];
                        double tempVehCapVar = param_.vehParam_[k].capVar_[aGeq.capId_];
                        int yId = sub2yId(k, i);
                        GRBVar tempBinaryYVar;
                        if (flagContinuous_) {
                            tempBinaryYVar = yrVar_[yId];
                        }
                        else {
                            tempBinaryYVar = yVar_[yId];
                        }
                        GRBLinExpr constraint = tempBinaryYVar * (- tempVehCapMu + tempTaskCapMu + betaRelated * sqrt(tempVehCapVar + tempTaskCapVar));
                        // TODO: Use this, need test
                        // GRBLinExpr constraint = tempBinaryYVar * (- tempVehCapMu + tempTaskCapMu + betaRelated * sqrt(tempVehCapVar + tempTaskCapVar))
                        //                         + aGeq.riskVar_.get(GRB_DoubleAttr_LB) * (1.0 - tempBinaryYVar);
                        std::string constraintName = "taskRiskConstraint[" + commonName + toString(TEAMPLANNER_VEHC, k) + "]";
                        model_->addConstr(aGeq.riskVar_ >= constraint, constraintName);
                    }
                }
                else {
                    // Cumulative capability
                    GRBLinExpr constraint = - aGeq.riskVar_ + aGeq.helpVar_;
                    std::string constraintName = "taskRiskConstraint[" + commonName + "]";
                    double multiplier = 1.0 / (static_cast<double>(param_.sampleNum_) * (1.0 - param_.taskBeta_));
                    for (int sampleId = 0; sampleId < param_.sampleNum_; sampleId++) {
                        constraint += multiplier * aGeq.sampleVar_[sampleId];
                        GRBLinExpr constraint1 = - aGeq.sampleVar_[sampleId] + aGeq.capSample_[sampleId] - aGeq.helpVar_;
                        std::string constraintName1 = "taskRiskSampleConstraint[" + commonName + std::to_string(sampleId) + "]";
                        for (int k = 0; k < vehNum_; k++) {
                            int yId = sub2yId(k, i);
                            constraint1 -= vehVar_[k].capSample_[aGeq.capId_][sampleId] * yVar_[yId];
                        }
                        model_->addConstr(constraint1 <= 0, constraintName1);
                    }
                    model_->addConstr(constraint == 0, constraintName);
                }
            }
        }
    }
}

void TeamPlanner::formTaskRiskCostLShaped() {
    double betaRelated = normalCDFInverse(param_.taskBeta_);
    for (int i = 0; i < taskNum_; i++) {
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                std::string commonName  = toString(TEAMPLANNER_TASK, i)     + ","
                                        + toString(TEAMPLANNER_AND , andId) + ","
                                        + toString(TEAMPLANNER_OR  , orId);
                if (param_.capType_[aGeq.capId_] == 1) {
                    // Non-cumulative capability
                    // GRBLinExpr constraint = - aGeq.riskVar_;
                    double tempTaskCapMu = param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                    double tempTaskCapVar = param_.taskParam_[i].reqFcn_[andId][orId].capVar_;
                    for (int k = 0; k < vehNum_; k++) {
                        double tempVehCapMu = param_.vehParam_[k].capVector_[aGeq.capId_];
                        double tempVehCapVar = param_.vehParam_[k].capVar_[aGeq.capId_];
                        int yId = sub2yId(k, i);
                        GRBVar tempBinaryYVar;
                        if (flagContinuous_) {
                            tempBinaryYVar = yrVar_[yId];
                        }
                        else {
                            tempBinaryYVar = yVar_[yId];
                        }
                        GRBLinExpr constraint = tempBinaryYVar * (- tempVehCapMu + tempTaskCapMu + betaRelated * sqrt(tempVehCapVar + tempTaskCapVar));
                        // TODO: Use this, need test
                        // GRBLinExpr constraint = tempBinaryYVar * (- tempVehCapMu + tempTaskCapMu + betaRelated * sqrt(tempVehCapVar + tempTaskCapVar))
                        //                         + aGeq.riskVar_.get(GRB_DoubleAttr_LB) * (1.0 - tempBinaryYVar);
                        std::string constraintName = "taskRiskConstraint[" + commonName + toString(TEAMPLANNER_VEHC, k) + "]";
                        model_->addConstr(aGeq.riskVar_ >= constraint, constraintName);
                    }
                }
                else {
                    //abcd
                    // Cumulative capability
                    if (param_.flagSolver_ == TEAMPLANNER_TASKLSHAPED || param_.flagSolver_ == TEAMPLANNER_CONTASKLSHAPED) {
                        double multiplier = 1.0 / (static_cast<double>(param_.sampleNum_) * (1.0 - param_.taskBeta_));
                        GRBLinExpr constraint = - aGeq.riskVar_ + aGeq.helpVar_ + multiplier * aGeq.lbVar_;
                        std::string constraintName = "taskRiskConstraint[" + commonName + "]";
                        model_->addConstr(constraint == 0, constraintName);
                    }
                    else {
                        // TEAMPLANNER_TASKNON
                        double tempTaskCapMu = param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                        GRBLinExpr constraint = - aGeq.riskVar_ + aGeq.lbVar_ + tempTaskCapMu;
                        std::string constraintName = "taskRiskConstraint[" + commonName + "]";
                        for (int k = 0; k < vehNum_; k++) {
                            int yId = sub2yId(k, i);
                            double tempVehCapMu = param_.vehParam_[k].capVector_[aGeq.capId_];
                            // double tempVehCapVar = param_.vehParam_[k].capVar_[aGeq.capId_];
                            constraint -= tempVehCapMu * yVar_[yId];
                        }
                        model_->addConstr(constraint == 0, constraintName);
                    }
                    // for (int sampleId = 0; sampleId < param_.sampleNum_; sampleId++) {
                    //     constraint += multiplier * aGeq.sampleVar_[sampleId];
                    //     GRBLinExpr constraint1 = - aGeq.sampleVar_[sampleId] + aGeq.capSample_[sampleId] - aGeq.helpVar_;
                    //     std::string constraintName1 = "taskRiskSampleConstraint[" + commonName + std::to_string(sampleId) + "]";
                    //     for (int k = 0; k < vehNum_; k++) {
                    //         int yId = sub2yId(k, i);
                    //         constraint1 -= vehVar_[k].capSample_[aGeq.capId_][sampleId] * yVar_[yId];
                    //     }
                    //     model_->addConstr(constraint1 <= 0, constraintName1);
                    // }
                }
            }
        }
    }
}

bool TeamPlanner::addTaskLShapedCut(double* yValue, std::vector<GRBLinExpr>& recourseCut) const {
    std::vector<double> yCoeff(vehNum_);
    double helperCoeff = 0.0;
    double rightCoeff = 0.0;
    bool flagCut = false;
    for (int i = 0; i < taskNum_; i++) {
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                const TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                if (param_.capType_[aGeq.capId_] == 1) {
                    continue;
                }
                std::fill(yCoeff.begin(), yCoeff.end(), 0.0);
                helperCoeff = 0.0;
                rightCoeff = 0.0;
                for (int sampleId = 0; sampleId < param_.sampleNum_; sampleId++) {
                    double tempValue = aGeq.capSample_[sampleId] - aGeq.helpValue_;
                    for (int k = 0; k < vehNum_; k++) {
                        int yId = sub2yId(k, i);
                        tempValue -= vehVar_[k].capSample_[aGeq.capId_][sampleId] * yValue[yId];
                    }
                    if (tempValue >= 0.0) {
                        for (int k = 0; k < vehNum_; k++) {
                            yCoeff[k] += vehVar_[k].capSample_[aGeq.capId_][sampleId];
                        }
                        helperCoeff += 1.0;
                        rightCoeff += aGeq.capSample_[sampleId];
                    }
                }
                double flagValue = - helperCoeff * aGeq.helpValue_ - aGeq.lbValue_ + rightCoeff;
                for (int k = 0; k < vehNum_; k++) {
                    int yId = sub2yId(k, i);
                    flagValue -= yCoeff[k] * yValue[yId];
                }
                if (flagValue > 1.0) { // Hard code 0.001: TODO, this should be larger when sample number is larger
                    // Add a new cut
                    flagCut = true;
                    GRBLinExpr aCut =  - helperCoeff * aGeq.helpVar_ - aGeq.lbVar_ + rightCoeff; // Can optimize: right handside reference
                    for (int k = 0; k < vehNum_; k++) {
                        int yId = sub2yId(k, i);
                        aCut -= yCoeff[k] * yVar_[yId];
                    }
                    recourseCut.push_back(aCut);
                }
            }
        }
    }
    return flagCut;
}

bool TeamPlanner::addTaskIntLShapedCut(double* yValue, std::vector<GRBLinExpr>& recourseCut) const {
    bool flagCut = false;
    double betaCVaRCoeff = normalCVaRCoefficient(param_.taskBeta_);
    for (int i = 0; i < taskNum_; i++) {
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                const TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                if (param_.capType_[aGeq.capId_] == 1) {
                    continue;
                }
                // double tempCapMu  = param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                double tempCapVar = param_.taskParam_[i].reqFcn_[andId][orId].capVar_;
                for (int k = 0; k < vehNum_; k++) {
                    int yId = sub2yId(k, i);
                    if (yValue[yId] > 0.5) {
                        // tempCapMu -= param_.vehParam_[k].capVector_[aGeq.capId_];
                        tempCapVar += param_.vehParam_[k].capVar_[aGeq.capId_];
                    }
                }
                double tempCapStd = sqrt(tempCapVar);
                double tempCost = betaCVaRCoeff * tempCapStd;
                if (aGeq.lbValue_ < tempCost - 0.0001) { // Hardcode: 0.0000001
                    // Add cut
                    flagCut = true;
                    GRBLinExpr aCut = - aGeq.lbVar_ + tempCost;
                    for (int k = 0; k < vehNum_; k++) {
                        int yId = sub2yId(k, i);
                        if (yValue[yId] > 0.5) {
                            aCut +=  tempCost * yVar_[yId] - tempCost;
                        }
                    }
                    recourseCut.push_back(aCut);
                }
                // else if (aGeq.lbValue_ > tempCost + 0.0001) {
                //     std::cout << "This should not happen!";
                // }
            }
        }
    }
    return flagCut;
}

bool TeamPlanner::getProbSuccess(double& riskCostNon, double& riskCostLin, const std::vector<std::vector<double>>* taskTeamDense) {
    riskCostNon = 0.0;
    riskCostLin = 0.0;
    double betaCVaRCoeff = normalCVaRCoefficient(param_.taskBeta_);
    double betaRelated = normalCDFInverse(param_.taskBeta_);
    for (int i = 0; i < taskNum_; i++) {
        taskVar_[i].probSuccess_ = 1.0;
        taskVar_[i].riskCost_ = 0.0;
        for (int andId = 0; andId < static_cast<int>(taskVar_[i].reqFcn_.size()); andId++) {
            for (int orId = 0; orId < static_cast<int>(taskVar_[i].reqFcn_[andId].size()); orId++) {
                TaskVarGeq& aGeq = taskVar_[i].reqFcn_[andId][orId];
                // double tempVaR = aGeq.helpVar_.get(GRB_DoubleAttr_X);
                if (param_.capType_[aGeq.capId_] == 1) {
                    // Non-cumulative capability
                    aGeq.probSuccess_ = 1.0;
                    aGeq.riskCost_ = 0.0;
                    double tempTaskCapMu = param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                    double tempTaskCapVar = param_.taskParam_[i].reqFcn_[andId][orId].capVar_;
                    for (int k = 0; k < vehNum_; k++) {
                        double tempVehCapMu = param_.vehParam_[k].capVector_[aGeq.capId_];
                        double tempVehCapVar = param_.vehParam_[k].capVar_[aGeq.capId_];
                        double tempBinaryYVar = 0.0;
                        double yValue = 0.0;
                        if (taskTeamDense == nullptr) {
                            int yId = sub2yId(k, i);
                            yValue = yVar_[yId].get(GRB_DoubleAttr_X);
                        }
                        else {
                            yValue = (*taskTeamDense)[i][k];
                        }
                        if (yValue < 0.0001) {
                            continue;
                        }
                        tempBinaryYVar = 1.0;
                        double tempCapStd = sqrt(tempVehCapVar + tempTaskCapVar);
                        double tempCapMu = - tempVehCapMu + tempTaskCapMu;
                        double aCapSample = tempBinaryYVar * (tempCapMu + betaRelated * tempCapStd);
                        if (aCapSample > aGeq.riskCost_) {
                            aGeq.riskCost_ = aCapSample;
                        }
                        // aGeq.probSuccess_ *= normalCDF(0.0, tempCapMu, tempCapStd );
                        double tempProbSucess = normalCDF(0.0, tempCapMu, tempCapStd );
                        if (tempProbSucess < aGeq.probSuccess_) {
                            aGeq.probSuccess_ = tempProbSucess;
                        }
                    }
                }
                else{
                    double tempCapMu  = param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                    double tempCapVar = param_.taskParam_[i].reqFcn_[andId][orId].capVar_;
                    for (int k = 0; k < vehNum_; k++) {
                        double yValue = 0.0;
                        if (taskTeamDense == nullptr) {
                            int yId = sub2yId(k, i);
                            yValue = yVar_[yId].get(GRB_DoubleAttr_X);
                        }
                        else {
                            yValue = (*taskTeamDense)[i][k];
                        }
                        if (yValue > 0.5) {
                            tempCapMu -= param_.vehParam_[k].capVector_[aGeq.capId_] * yValue;
                            tempCapVar += param_.vehParam_[k].capVar_[aGeq.capId_] * yValue * yValue;
                        }
                    }
                    double tempCapStd = sqrt(tempCapVar);
                    aGeq.probSuccess_ = normalCDF(0.0, tempCapMu, tempCapStd );
                    aGeq.riskCost_ = tempCapMu + betaCVaRCoeff * tempCapStd;
                }

                // std::cout << "betaCVaRCoeff * tempCapStd = " << betaCVaRCoeff * tempCapStd;
                // if (param_.flagSolver_ == TEAMPLANNER_TASKLSHAPED || param_.flagSolver_ == TEAMPLANNER_TASKNON) {
                //     std::cout << " == "  << aGeq.lbVar_.get(GRB_DoubleAttr_X);
                // }
                // std::cout << "\n";
                // std::cout << "aGeq.riskCost_ = " << aGeq.riskCost_;
                // if (flagTaskVarAdded_) {
                //     std::cout << " == "  << aGeq.riskVar_.get(GRB_DoubleAttr_X);
                // }
                // std::cout << "\n";
                // std::cout << i << ", " << andId << ", " << orId << "\t" << tempCapMu << "\t" << sqrt(tempCapVar) << "\n";
                taskVar_[i].probSuccess_ *= aGeq.probSuccess_;
                if (flagTaskVarAdded_) {
                    taskVar_[i].riskCost_ += aGeq.riskCost_ * aGeq.riskVar_.get(GRB_DoubleAttr_Obj);
                    double aCostLin = 0.0;
                    if (taskTeamDense == nullptr) {
                        aCostLin = aGeq.riskVar_.get(GRB_DoubleAttr_X);
                    }
                    else {
                        if (param_.capType_[aGeq.capId_] == 1) {
                            // Non-cumulative capability
                            aCostLin = aGeq.riskCost_;
                        }
                        else {
                            std::vector<double> aCapSample(param_.sampleNum_);
                            for (int sampleId = 0; sampleId < param_.sampleNum_; sampleId++) {
                                aCapSample[sampleId] = aGeq.capSample_[sampleId];
                                for (int k = 0; k < vehNum_; k++) {
                                    aCapSample[sampleId] -= (*taskTeamDense)[i][k] * vehVar_[k].capSample_[aGeq.capId_][sampleId];
                                }
                            }
                            double aVaR = 0.0;
                            double aCVaR = getCVaR(aCapSample, param_.taskBeta_, aVaR);
                            aCostLin = aCVaR;
                            // std::cout << "getProbSuccess: " << aGeq.riskVar_.get(GRB_DoubleAttr_X) << "\t" << aCVaR << "\n";
                        }
                    }
                    riskCostLin += aCostLin * aGeq.riskVar_.get(GRB_DoubleAttr_Obj);
                }
                else {
                    taskVar_[i].riskCost_ += aGeq.riskCost_ * param_.taskRiskPenalty_ / param_.taskParam_[i].reqFcn_[andId][orId].capReq_;
                }
            }
        }
        riskCostNon += taskVar_[i].riskCost_;
    }
    return true;
}
