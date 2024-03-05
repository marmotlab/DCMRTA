#ifndef LOCALMATH_HPP
#define LOCALMATH_HPP

#include <string>
#include <vector>

double normalCDF(double x, double mu, double sigma);
double normalCDFInverse(double p);
double normalCVaRCoefficient(double p);
std::string vecToString(const std::vector<double>& data, const std::string& delimiter = ",");
std::string vecToString(const std::vector<int>&    data, const std::string& delimiter = ",");

double vecSum(const std::vector<double>& data);
double vecMax(const std::vector<double>& data, int* index = nullptr);

double getCVaR(std::vector<double> data, double beta, double& VaR);

#endif //LOCALMATH_HPP