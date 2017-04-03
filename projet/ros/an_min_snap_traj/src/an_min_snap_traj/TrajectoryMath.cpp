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

    VectorXd polyval(VectorXd coefficients, VectorXd x) {
        VectorXd res = VectorXd::Zero(x.rows());
        for(int i = 0; i < x.rows(); ++i){
            res(i) = polyval(coefficients, x(i));
        }
        return res;
    }

    VectorXd polyder(VectorXd coefficients) {
        // We don't check if the vector is 1d because that
        // doesn't exist with eigen.
        VectorXd derived = VectorXd::Zero(coefficients.rows()-1);
        long highest_power = coefficients.rows()-1;
        for(int i = 0; i < highest_power; ++i) {
            derived(i) = coefficients(i) * (highest_power-i);
        }
        return derived;
    }

    Eigen::MatrixXd rot90(Eigen::MatrixXd m){
        MatrixXd r = m;
        for(int i = 0; i < m.rows(); ++i){
            r.row(i) = m.col(m.cols() - 1 - i);
        }
        return r;
    }

    Eigen::MatrixXd rot90(Eigen::MatrixXd m, int k){
        for(int i = 0; i < k; ++i) {
            m = rot90(m);
        }
        return m;
    }

    Eigen::VectorXd rightPadZeros(VectorXd vec, int s){
        if(vec.size() >= s) {
            return vec;
        } else {
            VectorXd padded(s);
            padded << vec , VectorXd::Zero(s-vec.size());
            return padded;
        }
    }

    Eigen::VectorXd leftPadZeros(Eigen::VectorXd vec, int s) {
        if( vec.size() >= s) {
            return vec;
        } else {
            VectorXd padded(s);
            padded << VectorXd::Zero(s-vec.size()), vec;
            return padded;
        }
    }

    Eigen::MatrixXd genCoefficientMatrix(int n, int r){
        int n_coeffs = n + 1;
        int n_poly = r + 1;
        MatrixXd coeffs = MatrixXd::Zero(n_poly, n_coeffs);
        VectorXd polynomial = VectorXd::Ones(n_coeffs);
        for(int i = 0; i < n_poly; ++i) {
            coeffs.row(i) = rightPadZeros(polynomial, n_coeffs);
            polynomial = polyder(polynomial);
        }
        return coeffs;
    }

    Eigen::VectorXd scalarPowered(double scalar, Eigen::VectorXd powers) {
        VectorXd res = VectorXd::Ones(powers.size());
        for(int i = 0; i < powers.size(); ++i) {
            res(i) = std::pow(scalar, powers(i));
        }
        return res;
    }
}