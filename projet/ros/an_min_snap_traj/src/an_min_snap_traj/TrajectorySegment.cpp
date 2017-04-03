//
// Created by andrephu-vannguyen on 02/04/17.
//

#include <vector>
#include <Eigen/Core>

#include "an_min_snap_traj/TrajectoryMath.hpp"
#include "an_min_snap_traj/TrajectorySegment.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    TrajectorySegment::TrajectorySegment(VectorXd *polynomial) {}

    TrajectorySegment::TrajectorySegment(VectorXd x, VectorXd y, VectorXd z) {}

    std::vector<Vector3d> TrajectorySegment::discretize(double dt) {
        return discretize(dt, 1);
    }

    std::vector<Vector3d> TrajectorySegment::discretize(double dt, double alpha) {
        // Not my best code...

        // linspace with dt
        double lo = 0.0;
        double hi = 1;
        double step = dt*alpha;
        double size = hi / dt;
        VectorXd times = VectorXd::LinSpaced(((hi-lo)/step) + 1,
                                             lo, lo+step*(size-1));

        VectorXd trajectory[State::STATE_COUNT];
        VectorXd polynomials[State::STATE_COUNT] =
                {polynomial_[0], polynomial_[1], polynomial_[2]};
        for(int der = 0; der <= Derivative::DER_ACCELERATION; ++der) {
            for(int state = 0; state < State::STATE_COUNT; ++state){
                polyval(polynomials[state], times);
            }
        }
    }
}
