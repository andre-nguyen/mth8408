//
// Created by andrephu-vannguyen on 02/04/17.
//

#ifndef TRAJECTORYSEGMENT_HPP_H
#define TRAJECTORYSEGMENT_HPP_H
#include <Eigen/Core>

#include "an_min_snap_traj/TrajectoryMath.hpp"

using namespace Eigen;

namespace an_min_snap_traj {

    class TrajectorySegment {
    public:
        TrajectorySegment(VectorXd polynomial[3]);
        TrajectorySegment(VectorXd x, VectorXd y, VectorXd z);

        std::vector<Vector3d> getPositionTrajectory();
        std::vector<Vector3d> getVelocityTrajectory();
        std::vector<Vector3d> getAccelerationTrajectory();

        std::vector<Vector3d> discretize(double dt);
        std::vector<Vector3d> discretize(double dt, double alpha);
    private:
        VectorXd polynomial_[State::STATE_COUNT]; // Non-dimensionalized trajectory segment
        std::vector<Vector3d> trajectory_[Derivative::DER_ACCELERATION];
    };
}

#endif //RAJECTORYSEGMENT_HPP_H
