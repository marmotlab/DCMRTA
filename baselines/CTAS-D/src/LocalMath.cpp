#include <LocalMath.hpp>
#include <cmath>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>

double normalPDF(double x) {
    return 0.398942280401433 * exp(-x*x/2.0);
}

double normalCDF(double x, double mu, double sigma) {
    if (sigma < 1e-6) {
        sigma = 1e-6;
    }
    double xx = (x - mu) / sigma;
    return std::erfc(-xx/std::sqrt(2))/2;
}

// This approximation is copied from https://www.johndcook.com/blog/cpp_phi_inverse/
double rationalApproximation(double t) {
    // Abramowitz and Stegun formula 26.2.23.
    // The absolute value of the error should be less than 4.5 e-4.
    double c[] = {2.515517, 0.802853, 0.010328};
    double d[] = {1.432788, 0.189269, 0.001308};
    return t - ((c[2]*t + c[1])*t + c[0]) / 
               (((d[2]*t + d[1])*t + d[0])*t + 1.0);
}

double normalCDFInverse(double p) {
    if (p <= 0.0 || p >= 1.0) {
        std::stringstream os;
        os << "Invalid input argument (" << p 
           << "); must be larger than 0 but less than 1.";
        throw std::invalid_argument( os.str() );
    }

    // See article above for explanation of this section.
    if (p < 0.5) {
        // F^-1(p) = - G^-1(p)
        return -rationalApproximation( sqrt(-2.0*log(p)) );
    }
    else {
        // F^-1(p) = G^-1(1-p)
        return rationalApproximation( sqrt(-2.0*log(1-p)) );
    }
}

double normalCVaRCoefficient(double p) {
    if (p > 0.5) {
        p = 1 - p;
    }
    return normalPDF(normalCDFInverse(p)) / p;
}

template<class T>
inline std::string templateToString(const std::vector<T>& data, const std::string& delimiter) {
    std::string str = "[";
    for (size_t i = 0; i < data.size(); i++) {
        if (i > 0) {
            str += delimiter +  " ";
        }
        str += std::to_string(data[i]);
    }
    str += "]";
    return str;
}

std::string vecToString(const std::vector<double>& data, const std::string& delimiter) {
    return templateToString(data, delimiter);
}

std::string vecToString(const std::vector<int>& data, const std::string& delimiter) {
    return templateToString(data, delimiter);
}

double vecSum(const std::vector<double>& data) {
    double sumValue = 0.0;
    for (size_t i = 0; i < data.size(); i++) {
        sumValue += data[i];
    }
    return sumValue;
}

double vecMax(const std::vector<double>& data, int* index) {
    int maxId = 0;
    double maxValue = data[0];
    for (size_t i = 0; i < data.size(); i++) {
        if (data[i] > maxValue) {
            maxValue = data[i];
            maxId = i;
        }
    }
    if (index != nullptr) {
        *index = maxId;
    }
    return maxValue;
}

double getCVaR(std::vector<double> data, double beta, double& VaR) {
    int N = static_cast<int>(data.size());
    int betaN = static_cast<int>(static_cast<double>(N-1) * beta + 0.5);
    std::nth_element(data.begin(), data.begin() + betaN, data.end());
    VaR = data[betaN];

    double CVaR = 0.0;
    for (int i = betaN+1; i < N; i++) {
        CVaR += data[i];
    }
    CVaR /= static_cast<double>(N - betaN - 1);

    return CVaR;
}
