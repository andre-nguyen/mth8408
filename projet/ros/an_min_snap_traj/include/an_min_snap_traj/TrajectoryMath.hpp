//
// Created by andre on 4/1/17.
//

#ifndef TRAJECTORYMATH_H
#define TRAJECTORYMATH_H

#include <Eigen/Core>

namespace an_min_snap_traj {
    enum Derivative {
        DER_POSITION = 0,
        DER_VELOCITY,
        DER_ACCELERATION,
        DER_JERK,
        DER_SNAP,
        DER_COUNT
    };

    enum State {
        STATE_X = 0,
        STATE_Y,
        STATE_Z,
        STATE_COUNT
    };

    /**
     * Custom implementation of polyval function just like
     * in matlab. e.g. if the input is [3 2 1] then the polynomial
     *  3x^2 + 2x + 1. So the lower the index, the higher the power.
     * @param coefficients of the polynomial
     * @param x point at which to evaluate the polynomial
     * @return
     */
    double polyval(Eigen::VectorXd coefficients, double x);

    /**
     * Custom implementation of polyval function just like
     * in matlab. e.g. if the input is [3 2 1] then the polynomial
     *  3x^2 + 2x + 1. So the lower the index, the higher the power.
     * @param coefficients of the polynomial
     * @param x points at which to evaluate the polynomial
     * @return A vector as long as x where is entry is the polyval of the
     * corresponding entry of x
     */
    Eigen::VectorXd polyval(Eigen::VectorXd coefficients, Eigen::VectorXd x);

    /**
     * Replicates the polyder function of matlab
     * @param coefficients
     * @return
     */
    Eigen::VectorXd polyder(Eigen::VectorXd coefficients);

    /**
     * Replicate matlab's rot90 function
     * @param m
     * @return
     */
    Eigen::MatrixXd rot90(Eigen::MatrixXd m);

    /**
     * Rotate matrix by 90 degrees k times
     * @param m matrix
     * @param k times
     * @return
     */
    Eigen::MatrixXd rot90(Eigen::MatrixXd m, int k);

    /**
     * Generates a coefficient matrix where each line are the coefficients
     * of a derivative of a polynomial of order n. Example for n = 6 and r = 4
     * we should get
     *        1     1     1     1     1     1     1
     *        6     5     4     3     2     1     0
     *       30    20    12     6     2     0     0
     *      120    60    24     6     0     0     0
     *      360   120    24     0     0     0     0
     * @param n Order of the polynomial
     * @param r Order of the highest derivative
     * @return
     */
    Eigen::MatrixXd genCoefficientMatrix(int n, int r);

    /**
     * Takes a vector of size n < s and pads it with
     * zeros on its right until it has s elements.
     * @param vec
     * @param s
     * @return
     */
    Eigen::VectorXd rightPadZeros(Eigen::VectorXd vec, int s);

    /**
     * Takes a vetor of size n < s and pads it with
     * zeros on its left until it has s elements
     * @param s
     * @return
     */
    Eigen::VectorXd leftPadZeros(Eigen::VectorXd vec, int s);

    /**
     * Takes a scalar and puts it to the power of powers.
     * e.g. scalar = 2 and powers = [3 2 1]
     *      result is [2^3 2^2 2^1]
     * @param scalar
     * @param powers
     * @return
     */
    Eigen::VectorXd scalarPowered(double scalar, Eigen::VectorXd powers);

    /**
     * Takes an eigen matrix and dumps it in rowmajor form into a buffer.
     * @param mat
     * @param buf
     */
    void eigenMat2buf(Eigen::MatrixXd mat, double buf[]);

    /**
     * Converts a vector of arrival times to segment times.
     * @param t Arrival times.
     * @return
     */
    Eigen::VectorXd time2segment(Eigen::VectorXd t);

    /**
     * Converts a vector of segment times to arrival times
     * @param T segment times
     * @return
     */
    Eigen::VectorXd segment2time(Eigen::VectorXd T);

    /**
     * Reallocates segmentimes
     * @param t Arrival times (first is always 0)
     * @param delta
     */
    Eigen::VectorXd segtimeRealloc(Eigen::VectorXd t, Eigen::VectorXd delta);
}

#endif //TRAJECTORYMATH_H
