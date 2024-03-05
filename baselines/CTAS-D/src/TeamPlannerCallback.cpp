#include <TeamPlannerCallback.hpp>
#include <cmath>

TeamPlannerCallback::TeamPlannerCallback() {
}

TeamPlannerCallback::TeamPlannerCallback(int varNum, GRBVar* var, TeamPlanner* planner, std::ofstream* logFile) {
    lastIter_ = -GRB_INFINITY;
    lastNode_ = -GRB_INFINITY;
    varNum_ = varNum;
    var_ = var;
    logFile_ = logFile;
    planner_ = planner;
}

TeamPlannerCallback::~TeamPlannerCallback() {
}

void TeamPlannerCallback::callback () {
    try {
        if (where == GRB_CB_MIPSOL) {
            if (planner_->param().flagSolver_ == TEAMPLANNER_SPR) {
                double* xValue = getSolution(planner_->xVar(), planner_->xVarNum());
                double* thetaValue = getSolution(planner_->thetaVar(), planner_->vehNum());
                std::vector<GRBLinExpr> recourseCut;
                std::vector<double> vehRecourseCost;
                planner_->addRecourseCut(xValue, thetaValue, recourseCut, vehRecourseCost);
                // std::cout << "Recourse Cut Size: " << recourseCut.size() << "\n";
                for (size_t i = 0; i < recourseCut.size(); i++) {
                    addLazy(recourseCut[i], GRB_LESS_EQUAL, 0.0);
                    // planner_->model()->addConstr(recourseCut[i], GRB_LESS_EQUAL, 0.0);
                }
                planner_->cutNum_ += static_cast<int>(recourseCut.size());
                // planner_->update();
                delete[] xValue;
                delete[] thetaValue;
            }
            else if (planner_->param().flagSolver_ == TEAMPLANNER_CCP) {
                double* xValue = getSolution(planner_->xVar(), planner_->xVarNum());
                std::vector<GRBLinExpr> ccpCut;
                planner_->addCCPCut(xValue, ccpCut);
                // std::cout << "CCP Cut Size: " << ccpCut.size() << "\n";
                for (size_t i = 0; i < ccpCut.size(); i++) {
                    addLazy(ccpCut[i], GRB_LESS_EQUAL, 0.0);
                    // planner_->model()->addConstr(ccpCut[i], GRB_LESS_EQUAL, 0.0);
                }
                planner_->cutNum_ += static_cast<int>(ccpCut.size());
                // planner_->update();
                delete[] xValue;
            }
            else if (planner_->param().flagSolver_ == TEAMPLANNER_TASKLSHAPED || planner_->param().flagSolver_ == TEAMPLANNER_CONTASKLSHAPED || planner_->param().flagSolver_ == TEAMPLANNER_TASKNON) {
                double* yValue = getSolution(planner_->yVar(), planner_->yVarNum());
                std::vector<TaskVar>* taskVar = planner_->taskVar();
                for (int i = 0; i < planner_->taskNum(); i++) {
                    for (int andId = 0; andId < static_cast<int>((*taskVar)[i].reqFcn_.size()); andId++) {
                        for (int orId = 0; orId < static_cast<int>((*taskVar)[i].reqFcn_[andId].size()); orId++) {
                            TaskVarGeq& aGeq = (*taskVar)[i].reqFcn_[andId][orId];
                            aGeq.helpValue_ = getSolution(aGeq.helpVar_);
                            aGeq.lbValue_ = getSolution(aGeq.lbVar_);
                        }
                    }
                }
                std::vector<GRBLinExpr> recourseCut;
                if (planner_->param().flagSolver_ == TEAMPLANNER_TASKLSHAPED || planner_->param().flagSolver_ == TEAMPLANNER_CONTASKLSHAPED) {
                    planner_->addTaskLShapedCut(yValue, recourseCut);
                }
                else {
                    // TEAMPLANNER_TASKNON
                    planner_->addTaskIntLShapedCut(yValue, recourseCut);
                }
                // std::cout << "recourseCut.size() = " << recourseCut.size() << "\n";
                for (size_t i = 0; i < recourseCut.size(); i++) {
                    addLazy(recourseCut[i], GRB_LESS_EQUAL, 0.0);
                }
                planner_->cutNum_ += static_cast<int>(recourseCut.size());
                delete[] yValue;
            }
            // std::cout << "NumConstrs: " << planner_->model_->get(GRB_IntAttr_NumConstrs) << "\n";

            // MIP solution callback
            // int nodeCount = (int) getDoubleInfo(GRB_CB_MIPSOL_NODCNT);
            // double objVal = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
            // double objBest = getDoubleInfo(GRB_CB_MIPSOL_OBJBST);
            // double objBound = getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
            // int solCount = getIntInfo(GRB_CB_MIPSOL_SOLCNT);
            // std::cout << "GRB_CB_MIPSOL: "
            //     << "**** New solution at node " << nodeCount << ", objVal " << objVal << ", " << objBest << ", " << ", sol " << solCount << "\n";
        }
        // else if (where == GRB_CB_MESSAGE) {
        //     // Message callback
        //     std::string msg = getStringInfo(GRB_CB_MSG_STRING);
        //     if (logFile_ != nullptr) {
        //         *logFile_ << msg;
        //     }
        // }
    }
    catch (GRBException e) {
        std::cout << "Error number: " << e.getErrorCode() << "\n";
        std::cout << e.getMessage() << "\n";
    }
    catch (...) {
        std::cout << "Error during callback" << "\n";
    }
}

// void TeamPlannerCallback::callback () {
//     try {
//         if (where == GRB_CB_POLLING) {
//             // Ignore polling callback
//         }
//         else if (where == GRB_CB_PRESOLVE) {
//             // Presolve callback
//             int colDels = getIntInfo(GRB_CB_PRE_COLDEL);
//             int rowDels = getIntInfo(GRB_CB_PRE_ROWDEL);
//             if (colDels || rowDels) {
//                 std::cout << "GRB_CB_PRESOLVE:"
//                     << colDels << " columns and " << rowDels << " rows are removed" << "\n";
//             }
//         }
//         else if (where == GRB_CB_SIMPLEX) {
//             // Simplex callback
//             double iterCount = getDoubleInfo(GRB_CB_SPX_ITRCNT);
//             if (iterCount - lastIter_ >= 100) {
//                 lastIter_ = iterCount;
//                 double objVal = getDoubleInfo(GRB_CB_SPX_OBJVAL);
//                 int isPert = getIntInfo(GRB_CB_SPX_ISPERT);
//                 double primalInfeasibility = getDoubleInfo(GRB_CB_SPX_PRIMINF);
//                 double dualInfeasibility = getDoubleInfo(GRB_CB_SPX_DUALINF);
//                 char ch;
//                 if (isPert == 0)        { ch = ' '; }
//                 else if (isPert == 1)   { ch = 'S'; }
//                 else                    { ch = 'P'; }
//                 std::cout << "GRB_CB_SIMPLEX: "
//                     << iterCount << " " << objVal << ch << " " << primalInfeasibility << " " << dualInfeasibility << "\n";
//             }
//         }
//         else if (where == GRB_CB_MIP) {
//             // General MIP callback
//             double nodeCount = getDoubleInfo(GRB_CB_MIP_NODCNT);
//             double objBest = getDoubleInfo(GRB_CB_MIP_OBJBST);
//             double objBound = getDoubleInfo(GRB_CB_MIP_OBJBND);
//             int solCount = getIntInfo(GRB_CB_MIP_SOLCNT);
//             if (nodeCount - lastNode_ >= 100) {
//                 lastNode_ = nodeCount;
//                 int actNodes = (int) getDoubleInfo(GRB_CB_MIP_NODLFT);
//                 int iterCount = (int) getDoubleInfo(GRB_CB_MIP_ITRCNT);
//                 int cutcnt = getIntInfo(GRB_CB_MIP_CUTCNT);
//                 std::cout << "GRB_CB_MIP: "
//                         << nodeCount << " " << actNodes << " " << iterCount
//                         << " " << objBest << " " << objBound << " "
//                         << solCount << " " << cutcnt << "\n";
//             }
//             // if (fabs(objBest - objBound) < 0.1 * (1.0 + fabs(objBest))) {
//             //     std::cout << "Stop early - 10% gap achieved" << "\n";
//             //     abort();
//             // }
//             if (nodeCount >= 10000 && solCount) {
//                 std::cout << "Stop early - 10000 nodes explored" << "\n";
//                 abort();
//             }
//         }
//         else if (where == GRB_CB_MIPSOL) {
//             // MIP solution callback
//             int nodeCount = (int) getDoubleInfo(GRB_CB_MIPSOL_NODCNT);
//             double objVal = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
//             double objBest = getDoubleInfo(GRB_CB_MIPSOL_OBJBST);
//             double objBound = getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
//             int solCount = getIntInfo(GRB_CB_MIPSOL_SOLCNT);
//             std::cout << "GRB_CB_MIPSOL: "
//                 << "**** New solution at node " << nodeCount << ", objVal " << objVal << ", " << objBest << ", " << ", sol " << solCount << "\n";
//             // double* x = getSolution(var_, varNum_);
//             // std::cout << ", x[0] = " << x[0] << " ****" << "\n";
//             // delete[] x;
//         }
//         else if (where == GRB_CB_MIPNODE) {
//             // MIP node callback
//             int nodeCount = (int) getDoubleInfo(GRB_CB_MIPNODE_NODCNT);
//             double objBest = getDoubleInfo(GRB_CB_MIPNODE_OBJBST);
//             double objBound = getDoubleInfo(GRB_CB_MIPNODE_OBJBND);
//             int solCount = getIntInfo(GRB_CB_MIPNODE_SOLCNT);
//             std::cout << "GRB_CB_MIPNODE: " << "**** New node " << nodeCount << ", bestVal " << objBest << ", " << ", sol " << solCount << "\n";
//             if (getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL) {
//                 double* x = getNodeRel(var_, varNum_);
//                 setSolution(var_, x, varNum_);
//                 delete[] x;
//             }
//         }
//         else if (where == GRB_CB_BARRIER) {
//             // Barrier callback
//             int iterCount = getIntInfo(GRB_CB_BARRIER_ITRCNT);
//             double primobj = getDoubleInfo(GRB_CB_BARRIER_PRIMOBJ);
//             double dualobj = getDoubleInfo(GRB_CB_BARRIER_DUALOBJ);
//             double priminf = getDoubleInfo(GRB_CB_BARRIER_PRIMINF);
//             double dualinf = getDoubleInfo(GRB_CB_BARRIER_DUALINF);
//             double cmpl = getDoubleInfo(GRB_CB_BARRIER_COMPL);
//             std::cout << "GRB_CB_BARRIER: "
//                 << iterCount << " " << primobj << " " << dualobj << " " << priminf << " " << dualinf << " " << cmpl << "\n";
//         }
//         else if (where == GRB_CB_MESSAGE) {
//             // Message callback
//             std::string msg = getStringInfo(GRB_CB_MSG_STRING);
//             if (logFile_ != nullptr) {
//                 *logFile_ << msg;
//             }
//         }
//     }
//     catch (GRBException e) {
//         std::cout << "Error number: " << e.getErrorCode() << "\n";
//         std::cout << e.getMessage() << "\n";
//     }
//     catch (...) {
//         std::cout << "Error during callback" << "\n";
//     }
// }
