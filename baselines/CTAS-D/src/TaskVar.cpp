#include <TaskVar.hpp>

TaskVarGeq::TaskVarGeq() {
    capId_ = -1;
    sampleVar_ = nullptr;
    probSuccess_ = -1.0;
    riskCost_ = 0.0;
    helpValue_ = 0.0;
    lbValue_ = 0.0;
}

TaskVarGeq::~TaskVarGeq() {
    clear();
}

void TaskVarGeq::clear() {
    capId_ = -1;
    probSuccess_ = -1.0;
    riskCost_ = 0.0;
    helpValue_ = 0.0;
    lbValue_ = 0.0;
    capSample_.clear();
    if (sampleVar_ != nullptr) {
        delete[] sampleVar_;
        sampleVar_ = nullptr;
    }
}

TaskVar::TaskVar() {
    probSuccess_ = -1.0;
    riskCost_ = 0.0;
}

TaskVar::~TaskVar() {
    clear();
}

void TaskVar::clear() {
    probSuccess_ = -1.0;
    riskCost_ = 0.0;
    for (size_t i = 0; i < reqFcn_.size(); i++) {
        reqFcn_[i].clear();
    }
    reqFcn_.clear();
}

void TaskVar::genSample(const TaskParam& taskParam, std::default_random_engine& generator, int sampleNum, RandomType flagRandom) {
    reqFcn_.resize(taskParam.reqFcn_.size());
    for (int andId = 0; andId < static_cast<int>(taskParam.reqFcn_.size()); andId++) {
        reqFcn_[andId].resize(taskParam.reqFcn_[andId].size());
        for (int orId = 0; orId < static_cast<int>(taskParam.reqFcn_[andId].size()); orId++) {
            reqFcn_[andId][orId].capId_ = taskParam.reqFcn_[andId][orId].capId_;
            reqFcn_[andId][orId].capSample_.resize(sampleNum);
            if (flagRandom == RandomType::TEAMPLANNER_READ) {
                continue;
            }
            // Initialize distribution
            double capMean = taskParam.reqFcn_[andId][orId].capReq_;
            double capStd = sqrt(taskParam.reqFcn_[andId][orId].capVar_);
            std::normal_distribution<double> capDistribution(capMean, capStd);
            // Gnerate samples
            for (int s = 0; s < sampleNum; s++) {
                reqFcn_[andId][orId].capSample_[s] = capDistribution(generator);
            }
        }
    }
}

std::string TaskVar::toString() const {
    std::string str = "";
    for (int andId = 0; andId < static_cast<int>(reqFcn_.size()); andId++) {
        for (int orId = 0; orId < static_cast<int>(reqFcn_[andId].size()); orId++) {
            str += std::to_string(reqFcn_[andId][orId].capId_) + ",\t";
            for (size_t s = 0; s < reqFcn_[andId][orId].capSample_.size(); s++) {
                if (s > 0) {
                    str += ",\t";
                }
                str += std::to_string(reqFcn_[andId][orId].capSample_[s]);
            }
            str += "\n";
        }
    }
    return str;
}
