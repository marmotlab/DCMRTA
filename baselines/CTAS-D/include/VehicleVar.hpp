#ifndef TEAMPLANNER_VEHICLEVAR_HPP
#define TEAMPLANNER_VEHICLEVAR_HPP
#include <vector>
#include <string>
#include <random>
#include <VehicleParam.hpp>
#include <ConstantDef.hpp>

class VehicleVar
{
public:
    std::vector<std::vector<double>> capSample_;

public:
    VehicleVar(/* args */);
    ~VehicleVar();
    void clear();
    void genSample(const VehicleParam& vehParam, std::default_random_engine& generator, int sampleNum, RandomType flagRandom);
    std::string toString() const;
};

#endif //TEAMPLANNER_VEHICLEVAR_HPP