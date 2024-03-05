#ifndef TEAMPLANNER_VEHICLEPARAM_HPP
#define TEAMPLANNER_VEHICLEPARAM_HPP
#include <vector>
#include <string>

class VehicleParam
{
public:
    double engCap_;
    double engCost_;
    bool flagDistCost_;
    std::vector<double> capVector_;
    std::vector<double> capVar_;
    std::vector<int> capSampleNum_;

public:
    VehicleParam(/* args */);
    ~VehicleParam();
    std::string toString() const;
};

#endif //TEAMPLANNER_VEHICLEPARAM_HPP