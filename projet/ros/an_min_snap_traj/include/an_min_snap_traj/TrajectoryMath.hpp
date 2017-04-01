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
     * Replicates the polyder function of matlab
     * @param coefficients
     * @return
     */
    Eigen::VectorXd polyder(Eigen::VectorXd coefficients);
}

#endif //TRAJECTORYMATH_H
