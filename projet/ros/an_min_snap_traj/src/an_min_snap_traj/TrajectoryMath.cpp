//
// Created by andre on 4/1/17.
//

#include <cmath>
#include <Eigen/Core>
#include <iostream>
#include "an_min_snap_traj/TrajectoryMath.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    double polyval(VectorXd coefficients, double x) {
        double sum = 0;
        // iterate on everything except the last element
        for(int i = 0; i < coefficients.rows()-1; ++i){
            long n = coefficients.rows()-1;
            sum += coefficients(i) * std::pow(x, n-i);
        }
        sum += coefficients.tail(1)(0);
        return sum;
    }

    VectorXd polyder(VectorXd coefficients) {
        // We don't check if the vector is 1d because that
        // doesn't exist with eigen.
        VectorXd derived = VectorXd::Zero(coefficients.rows()-1);
        long highest_power = coefficients.rows()-1;
        for(int i = 0; i < highest_power-1; ++i) {
            derived(i) = coefficients(i) * (highest_power-i);
        }
        return derived;
    }
}