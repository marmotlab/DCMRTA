#include <TaskParam.hpp>
#include <math.h>

TaskReqGeq::TaskReqGeq() {
    geq_ = true;
    capId_ = -1;
    capReq_ = 0.0;
    capVar_ = 0.0;
}

TaskReqGeq::TaskReqGeq(int capId, double capReq, double capVar, bool geq) {
    geq_ = geq;
    capId_ = capId;
    capReq_ = capReq;
    capVar_ = capVar;
}

std::string TaskReqGeq::toString() const {
    std::string printString = "a" + std::to_string(capId_);
    if (geq_) {
        printString += ">";
    }
    else {
        printString += "<";
    }
    printString += std::to_string(static_cast<double>(capReq_ + 0.00));
    if (capVar_ > 1e-6) {
        printString += ", " + std::to_string(sqrt(capVar_));
    }
    return printString;
}

TaskReqGeq::~TaskReqGeq() {
}


TaskParam::TaskParam(/* args */) {
}

TaskParam::~TaskParam() {
}

std::string TaskParam::toString() const {
    std::string printString = "";
    for (int andId = 0; andId < static_cast<int>(reqFcn_.size()); andId++) {
        if (andId > 0) {
            printString += " & ";
        }
        printString += "(";
        for (int orId = 0; orId < static_cast<int>(reqFcn_[andId].size()); orId++) {
            if (orId > 0) {
                printString += " || ";
            }
            printString += reqFcn_[andId][orId].toString();
        }
        printString += ")";
    }
    return printString;
}

