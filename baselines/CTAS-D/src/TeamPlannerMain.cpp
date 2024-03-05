#include <TeamPlanner.hpp>

bool TeamPlanner::formProblem(const std::string& paramFile) {
    clear();

    getParams(paramFile);
    getGraph(param_.vehNum_, param_.taskNum_+2*param_.vehNum_, param_.graphFile_);
    initializeNum();
    initializeModel();

    formVarNameCost();
    formCommonModel();

    switch (param_.flagSolver_) {
    case TEAMPLANNER_SPR:
        formDeterministicEnergy();
        // std::cout << "before: " << model_->get(GRB_IntParam_LazyConstraints) << "\n";
        model_->set(GRB_IntParam_LazyConstraints, 1);
        // std::cout << "after : " << model_->get(GRB_IntParam_LazyConstraints) << "\n";
        break;
    case TEAMPLANNER_CCP:
        formDeterministicEnergy();
        model_->set(GRB_IntParam_LazyConstraints, 1);
        break;
    case TEAMPLANNER_SPRITER:
        formDeterministicEnergy();
        break;
    case TEAMPLANNER_CCPCONE:
        formDeterministicEnergy();
        formCCPEnergy();
        break;
    case TEAMPLANNER_CCPITER:
        formDeterministicEnergy();
        break;
    case TEAMPLANNER_TASKLIN:
        formDeterministicEnergy();
        formTaskRiskCost();
        break;
    case TEAMPLANNER_TASKLSHAPED:
        model_->set(GRB_IntParam_LazyConstraints, 1);
        formDeterministicEnergy();
        formTaskRiskCostLShaped();
        break;
    case TEAMPLANNER_TASKNON:
        model_->set(GRB_IntParam_LazyConstraints, 1);
        formDeterministicEnergy();
        formTaskRiskCostLShaped();
        break;
    case TEAMPLANNER_CONDET:
        formContinuousModel();
        formContinuousDetEnergy();
        break;
    case TEAMPLANNER_CONTASKLIN:
        formContinuousModel();
        formContinuousDetEnergy();
        formTaskRiskCost();
        break;
    case TEAMPLANNER_CONTASKLSHAPED:
        model_->set(GRB_IntParam_LazyConstraints, 1);
        formContinuousModel();
        formContinuousDetEnergy();
        formTaskRiskCostLShaped();
        break;
    default: // TEAMPLANNER_DET
        formDeterministicEnergy();
        break;
    }

    return true;
}

bool TeamPlanner::optimize() {
    flagSuccess_ = true;
    flagOptimized_ = true;
    std::ofstream logfile("callback.log");
    std::ofstream* logfilePtr = nullptr;
    if (logfile.is_open()) {
        logfilePtr = &logfile;
    }
    else {
        std::cout << "Cannot open callback.log for callback message" << std::endl;
    }
    int numvars = model_->get(GRB_IntAttr_NumVars);
    // GRBVar* vars = xVar_;
    if (param_.flagSolver_ == TEAMPLANNER_SPR || param_.flagSolver_ == TEAMPLANNER_CCP || param_.flagSolver_ == TEAMPLANNER_TASKLSHAPED || param_.flagSolver_ == TEAMPLANNER_TASKNON || param_.flagSolver_ == TEAMPLANNER_CONTASKLSHAPED) {
        TeamPlannerCallback plannerCallback = TeamPlannerCallback(numvars, xVar_, this, logfilePtr);
        model_->setCallback(&plannerCallback);
    }

    double startTime = GetTime(); // Mark the start time
    // printf("param_.flagSolver_ = %d\n", param_.flagSolver_);
    if (param_.flagSolver_ == TEAMPLANNER_SPRITER || param_.flagSolver_ == TEAMPLANNER_CCPITER) {
        if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
            initializeBestSolution();
        }
        for (int iterCut = 0; iterCut < 1000; iterCut++) {
            // Set time limit
            if (param_.solverMaxTime_ > 0.001) {
                double currTime = GetTime() - startTime;
                if (currTime > param_.solverMaxTime_) {
                    if (param_.flagSolver_ == TEAMPLANNER_CCPITER) {
                        flagSuccess_ = false;
                    }
                    break;
                }
                double smallerTime = param_.solverMaxTime_ - currTime;
                if (smallerTime > param_.solverIterMaxTime_) {
                    smallerTime = param_.solverIterMaxTime_;
                }
                model_->set(GRB_DoubleParam_TimeLimit, smallerTime);
            }

            model_->optimize();

            if (model_->get(GRB_IntAttr_SolCount) <= 0) {
                if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
                    if (bestSolution_.totalCost_ > 1e11) {
                        flagSuccess_ = false;
                        flagOptimized_ = false;
                        // copyToBestSolution(-1.0);
                        break;
                    }
                }
                else if (param_.flagSolver_ == TEAMPLANNER_CCPITER) {
                    flagSuccess_ = false;
                    break;
                }
            }

            std::vector<GRBLinExpr> newCut;

            bool flagNewCut = false;
            if (param_.flagSolver_ == TEAMPLANNER_SPRITER) {
                double* xValue = nullptr;
                double* thetaValue = nullptr;
                std::vector<double> vehRecourseCost;
                flagNewCut = addRecourseCut(xValue, thetaValue, newCut, vehRecourseCost);
                copyPreviousSolution(vehRecourseCost);
                double energyCost, timeCost, otherCost, fCost, thetaCost, totalCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost;
                getCost(vehRecourseCost, energyCost, timeCost, otherCost, fCost, thetaCost, recourseCost, oriLinCost, oriNonCost, oriRiskLinCost, oriRiskNonCost);
                totalCost = oriNonCost;
                if (totalCost < bestSolution_.totalCost_) {
                    copyToBestSolution(totalCost);
                }
            }
            else if (param_.flagSolver_ == TEAMPLANNER_CCPITER) {
                double* xValue = nullptr;
                flagNewCut = addCCPCut(xValue, newCut);
            }
            if (!flagNewCut) {
                break;
            }
            std::cout << "newCut.size() = " << newCut.size() << "iter = " << iterCut << "\n";
            for (size_t i = 0; i < newCut.size(); i++) {
                std::string constraintName = "newCut[" + std::to_string(iterCut) + "," + std::to_string(i) + "]";
                model_->addConstr(newCut[i] <= 0, constraintName);
            }
            cutNum_ += static_cast<int>(newCut.size());
        }
        constraintNum_ = model_->get(GRB_IntAttr_NumConstrs) - cutNum_;
    }
    else {
        model_->optimize();
        constraintNum_ = model_->get(GRB_IntAttr_NumConstrs);
        if (model_->get(GRB_IntAttr_SolCount) <= 0) {
            flagSuccess_ = false;
            flagOptimized_ = false;
        }
    }
    solverTime_ = GetTime() - startTime;

    logfile.close();

    return flagOptimized_;
}
