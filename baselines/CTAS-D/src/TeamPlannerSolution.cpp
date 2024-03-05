#include <TeamPlanner.hpp>

TeamPlannerSolution::TeamPlannerSolution() {
    totalCost_ = 1e15;
    xVar_ = nullptr;
    yVar_ = nullptr;
    zVar_ = nullptr;
    qVar_ = nullptr;
    thetaVar_ = nullptr;
    alphaVar_ = nullptr;
    wVar_ = nullptr;
}

TeamPlannerSolution::~TeamPlannerSolution() {
    clear();
}

void TeamPlannerSolution::clear() {
    totalCost_ = 1e15;
    if (xVar_ != nullptr) {
        delete[] xVar_;
        xVar_ = nullptr;
    }
    if (yVar_ != nullptr) {
        delete[] yVar_;
        yVar_ = nullptr;
    }
    if (zVar_ != nullptr) {
        delete[] zVar_;
        zVar_ = nullptr;
    }
    if (qVar_ != nullptr) {
        delete[] qVar_;
        qVar_ = nullptr;
    }
    if (thetaVar_ != nullptr) {
        delete[] thetaVar_;
        thetaVar_ = nullptr;
    }
    if (alphaVar_ != nullptr) {
        delete[] alphaVar_;
        alphaVar_ = nullptr;
    }
    if (wVar_ != nullptr) {
        delete[] wVar_;
        wVar_ = nullptr;
    }
}

void TeamPlannerSolution::initialize(int xVarNum, int yVarNum, int zVarNum, int qVarNum, int thetaVarNum, int alphaVarNum, int wVarNum) {
    clear();
    xVar_ = new double[xVarNum]();
    yVar_ = new double[yVarNum]();
    zVar_ = new double[zVarNum]();
    qVar_ = new double[qVarNum]();
    thetaVar_ = new double[thetaVarNum]();
    alphaVar_ = new double[alphaVarNum]();
    wVar_ = new double[wVarNum]();
}
