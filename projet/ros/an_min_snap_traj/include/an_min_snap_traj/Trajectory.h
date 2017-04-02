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

    private:
        VectorXd traj_[3];
    };
}

#endif //TRAJECTORY_H
