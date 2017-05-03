/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2017 Mobile Robotics and Autonomous Systems Laboratory (MRASL),
 * Polytechnique Montreal. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include <vector>
#include <Eigen/Core>

#include "an_min_snap_traj/TrajectoryMath.hpp"
#include "an_min_snap_traj/TrajectorySegment.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    TrajectorySegment::TrajectorySegment(VectorXd *polynomial) {
        t0_ = 0;        //non dimensionalized time
        tend_ = 1;
    }

    TrajectorySegment::TrajectorySegment(double t0, double tend, double alpha, VectorXd *polynomial) :
        t0_(t0), tend_(tend), alpha_(alpha)
    {
        polynomial_[0] = polynomial[0];
        polynomial_[1] = polynomial[1];
        polynomial_[2] = polynomial[2];
    }

    TrajectorySegment::TrajectorySegment(double t0, double tend, double alpha, VectorXd x, VectorXd y, VectorXd z) :
            t0_(t0), tend_(tend), alpha_(alpha)
    {
        polynomial_[0] = x;
        polynomial_[1] = y;
        polynomial_[2] = z;
    }

    std::vector<Vector3d> TrajectorySegment::getTraj(Derivative der) {
        return trajectory_[der];
    }

    std::vector<Vector3d> TrajectorySegment::discretize(double dt){
        // prepare scale factor
        // XXX Magic values, 7-> n_coeffs
        VectorXd alpha_vec = scalarPowered((1.0/alpha_), VectorXd::LinSpaced(7,0,6).reverse());


        // Copy polynomials because we need to derive them and we need to alpha
        // scale them since they are the solutions to the non dimensionalized problem';
        VectorXd polynomials[State::STATE_COUNT][Derivative::DER_ACCELERATION+1];
        for(int state = 0; state < State::STATE_COUNT; ++state){
            polynomials[state][Derivative::DER_POSITION] =
                    polynomial_[state].cwiseProduct(alpha_vec);
        }

        // Generate derivatives
        for(int der = 1; der <= Derivative::DER_ACCELERATION; ++der) {
            for(int state = 0; state < State::STATE_COUNT; ++state){
                polynomials[state][der] = polyder(polynomials[state][der-1]);
            }
        }

        // Evaluate the trajectory
        // linspace with dt
        double lo = t0_;
        double hi = tend_ * alpha_;
        double step = dt*alpha_;
        double size = hi / step;
        VectorXd times = VectorXd::LinSpaced(((hi-lo)/step) + 1,
                                             lo, lo+step*(size-1));
        for(double t = t0_; t < tend_*alpha_; t = t + (dt*alpha_)) {
            for (int der = 0; der <= Derivative::DER_ACCELERATION; ++der) {
                Vector3d point;
                double scaledt = (t - t0_) / (tend_ - t0_);
                scaledt = scaledt * alpha_;
                for (int state = 0; state < State::STATE_COUNT; ++state) {
                    point(state) = polyval(polynomials[state][der], scaledt);
                }
                trajectory_[der].push_back(point);
            }
        }
        return trajectory_[DER_POSITION];
    }
}
