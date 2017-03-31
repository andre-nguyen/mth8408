#include <Eigen/Dense>

#include "an_min_snap_traj/TrajectoryGenerator.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    TrajectoryGenerator::TrajectoryGenerator()
    {

    }

    int TrajectoryGenerator::getNumWaypoints() {
        return keyframes_.size() - 1;
    }

    void TrajectoryGenerator::buildCostMatrix(int dim) {
        //MatrixXd H = H_[dim];
        int wps = getNumWaypoints();
        for(int wp = 0; wp < wps; ++wp) {
            MatrixXd H = MatrixXd::Zero(n_coeffs_, n_coeffs_);
            double t0 = keyframes_[wp].getTime();
            double tend = keyframes_[wp+1].getTime();
            for(int i = 0; i < n_; ++i){
                for(int j = 0; j < n_; ++j){
                    if(i >= k_r_ && j >= k_r_) {

                    }
                }
            }
        }


    }
}
