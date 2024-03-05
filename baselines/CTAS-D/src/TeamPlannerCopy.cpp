#include <TeamPlanner.hpp>

void TeamPlanner::copyPreviousSolution(const std::vector<double>& vehRecourseCost) {
    for (int i = 0; i < xVarNum_; i++) {
        double tempValue = xVar_[i].get(GRB_DoubleAttr_X);
        xVar_[i].set(GRB_DoubleAttr_Start, tempValue);
    }
    for (int i = 0; i < yVarNum_; i++) {
        double tempValue = yVar_[i].get(GRB_DoubleAttr_X);
        yVar_[i].set(GRB_DoubleAttr_Start, tempValue);
    }
    for (int i = 0; i < zVarNum_; i++) {
        double tempValue = zVar_[i].get(GRB_DoubleAttr_X);
        zVar_[i].set(GRB_DoubleAttr_Start, tempValue);
    }
    for (int i = 0; i < qVarNum_; i++) {
        double tempValue = qVar_[i].get(GRB_DoubleAttr_X);
        qVar_[i].set(GRB_DoubleAttr_Start, tempValue);
    }
    for (size_t i = 0; i < alphaVar_.size(); i++) {
        double tempValue = alphaVar_[i].get(GRB_DoubleAttr_X);
        alphaVar_[i].set(GRB_DoubleAttr_Start, tempValue);
    }
    for (size_t i = 0; i < wVar_.size(); i++) {
        double tempValue = wVar_[i].get(GRB_DoubleAttr_X);
        wVar_[i].set(GRB_DoubleAttr_Start, tempValue);
    }
    if (thetaVar_ != nullptr) {
        for (int i = 0; i < vehNum_; i++) {
            thetaVar_[i].set(GRB_DoubleAttr_Start, vehRecourseCost[i]);
        }
    }
}

void TeamPlanner::initializeBestSolution() {
    bestSolution_.initialize(xVarNum_, yVarNum_, zVarNum_, qVarNum_, vehNum_, alphaVar_.size(), wVar_.size());
}

void TeamPlanner::copyToBestSolution(double totalCost) {
    bestSolution_.totalCost_ = totalCost;
    for (int i = 0; i < xVarNum_; i++) {
        bestSolution_.xVar_[i] = xVar_[i].get(GRB_DoubleAttr_X);
    }
    for (int i = 0; i < yVarNum_; i++) {
        bestSolution_.yVar_[i] = yVar_[i].get(GRB_DoubleAttr_X);
    }
    for (int i = 0; i < zVarNum_; i++) {
        bestSolution_.zVar_[i] = zVar_[i].get(GRB_DoubleAttr_X);
    }
    for (int i = 0; i < qVarNum_; i++) {
        bestSolution_.qVar_[i] = qVar_[i].get(GRB_DoubleAttr_X);
    }
    for (size_t i = 0; i < alphaVar_.size(); i++) {
        bestSolution_.alphaVar_[i] = alphaVar_[i].get(GRB_DoubleAttr_X);
    }
    for (size_t i = 0; i < wVar_.size(); i++) {
        bestSolution_.wVar_[i] = wVar_[i].get(GRB_DoubleAttr_X);
    }
    if (thetaVar_ != nullptr) {
        for (int i = 0; i < vehNum_; i++) {
            bestSolution_.thetaVar_[i] = thetaVar_[i].get(GRB_DoubleAttr_X);
        }
    }
}
