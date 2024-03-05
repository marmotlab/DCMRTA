#include <VehicleVar.hpp>
#include <math.h>

VehicleVar::VehicleVar() {
}

VehicleVar::~VehicleVar() {
    clear();
}

void VehicleVar::clear() {
    for (size_t i = 0; i < capSample_.size(); i++) {
        capSample_[i].clear();
    }
    capSample_.clear();
}

void VehicleVar::genSample(const VehicleParam& vehParam, std::default_random_engine& generator, int sampleNum, RandomType flagRandom) {
    clear();
    capSample_.resize(vehParam.capVector_.size());
    for (size_t a = 0; a < vehParam.capVector_.size(); a++) {
        capSample_[a].resize(sampleNum);
        if (flagRandom == RandomType::TEAMPLANNER_READ) {
            continue;
        }
        // Initialize distribution
        double capMean = vehParam.capVector_[a];
        double capStd = sqrt(vehParam.capVar_[a]);
        std::normal_distribution<double> capDistribution(capMean, capStd);
        // Gnerate samples
        for (int s = 0; s < sampleNum; s++) {
            capSample_[a][s] = capDistribution(generator);
        }
    }
}

std::string VehicleVar::toString() const {
    std::string printString = "";
    for (size_t a = 0; a < capSample_.size(); a++) {
        for (size_t s = 0; s < capSample_[a].size(); s++) {
            if (s > 0) {
                printString += ",\t";
            }
            printString += std::to_string(capSample_[a][s]);
        }
        printString += "\n";
    }
    return printString;
}
