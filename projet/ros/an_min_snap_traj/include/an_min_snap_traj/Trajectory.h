//
// Created by andre on 4/2/17.
//

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Core>

using namespace Eigen;

namespace an_min_snap_traj {
    class Trajectory {
    public:
        Trajectory();
        Trajectory(double dt);
        Trajectory(double dt, double alpha);

        VectorXd getTrajectory(int dim, int der);

    private:
        VectorXd polynomials_[3];   // n+1 x 1 vector of polynomial coefficients
        double dt_;                 // Delta time for discretization
        double alpha_;              // Time scale parameter

        bool isDiscretized;
        VectorXd trajs_[3][4];
    };
}

#endif //TRAJECTORY_H
