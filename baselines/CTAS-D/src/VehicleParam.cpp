#include <VehicleParam.hpp>
#include <math.h>

VehicleParam::VehicleParam() {
}

VehicleParam::~VehicleParam() {
}

std::string VehicleParam::toString() const {
    std::string printString = "(" + std::to_string(engCap_) + "| ";
    for(int capId = 0; capId < static_cast<int>(capVector_.size()); capId++) {
        if (capId > 0) {
            printString += ",\t";
        }
        printString += std::to_string( static_cast<int>(capVector_[capId]+0.01) );
        // printString += "-" + std::to_string( sqrt(capVar_[capId]) ); // Print vehicle capability standard deviation
        printString += "-" + std::to_string(capSampleNum_[capId]); // Print sample numbers
    }
    printString += ")";
    return printString;
}

