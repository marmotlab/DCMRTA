#include <TeamPlanner.hpp>

TeamPlanner::TeamPlanner()
{
    env_ = nullptr;
    model_ = nullptr;
    xVar_ = nullptr;
    yVar_ = nullptr;
    zVar_ = nullptr;
    qVar_ = nullptr;
    thetaVar_ = nullptr;
    xrVar_ = nullptr;
    yrVar_ = nullptr;
    gVar_ = nullptr;
    vehTypeNum_ = 0;
    vehNum_ = 0;
    taskNum_ = 0;
    capNum_ = 0;
    edgeNum_ = 0;
    nodeNum_ = 0;
    xVarNum_ = 0;
    yVarNum_ = 0;
    zVarNum_ = 0;
    qVarNum_ = 0;
    gVarNum_ = 0;
	startTime_ = std::chrono::system_clock::now();
    prevTime_ = 0.0;
    solverTime_ = -1.0;
    constraintNum_ = 0;
    flagOptimized_ = false;
    flagSuccess_ = false;
    flagTaskVarAdded_ = false;
    flagContinuous_ = false;
    cutNum_ = 0;
    // generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

TeamPlanner::~TeamPlanner() {
    clear();
}

void TeamPlanner::clear() {
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
    if (xrVar_ != nullptr) {
        delete[] xrVar_;
        xrVar_ = nullptr;
    }
    if (yrVar_ != nullptr) {
        delete[] yrVar_;
        yrVar_ = nullptr;
    }
    if (gVar_ != nullptr) {
        delete[] gVar_;
        gVar_ = nullptr;
    }
    if (model_ != nullptr) {
        delete model_;
        model_ = nullptr;
    }
    if (env_ != nullptr) {
        delete env_;
        env_ = nullptr;
    }
    alphaVar_.clear();
    wVar_.clear();
    bestSolution_.clear();
    graph_.clear();
    edgeOffset_.clear();
    sumCap_.clear();
    vehTypeNum_ = 0;
    vehNum_ = 0;
    taskNum_ = 0;
    capNum_ = 0;
    edgeNum_ = 0;
    nodeNum_ = 0;
    xVarNum_ = 0;
    yVarNum_ = 0;
    zVarNum_ = 0;
    qVarNum_ = 0;
    gVarNum_ = 0;
	// startTime_ = std::chrono::system_clock::now();
    prevTime_ = 0.0;
    solverTime_ = -1.0;
    constraintNum_ = 0;
    flagOptimized_ = false;
    flagSuccess_ = false;
    flagTaskVarAdded_ = false;
    flagContinuous_ = false;
    cutNum_ = 0;
    vehVar_.clear();
    taskVar_.clear();
}

double TeamPlanner::GetTime() const {
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	int passedDuration = (int) std::chrono::duration_cast <std::chrono::milliseconds> (now-startTime_).count();
	return (( (double)passedDuration ) / 1000.0);
}

double TeamPlanner::GetTimeDuration(){
	double currTime = GetTime();
	double timeDuration = currTime - prevTime_;
	prevTime_ = currTime;
	return timeDuration;
}
