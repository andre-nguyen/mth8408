#include <cmath>
#include <Eigen/Dense>

#include "an_min_snap_traj/TrajectoryGenerator.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    TrajectoryGenerator::TrajectoryGenerator() {

    }

    unsigned long TrajectoryGenerator::getNumWaypoints() {
        return keyframes_.size() - 1;
    }

    void TrajectoryGenerator::buildCostMatrix(int dim) {
        int h_size = n_coeffs_ * getNumWaypoints();
        H_[dim] = MatrixXd::Zero()
        unsigned long wps = getNumWaypoints();
        for (int wp = 0; wp < wps; ++wp) {
            MatrixXd H = MatrixXd::Zero(n_coeffs_, n_coeffs_);
            for (int i = 0; i < n_; ++i) {
                for (int j = 0; j < n_; ++j) {
                    if (i >= k_r_ && j >= k_r_) {
                        double cum_mul = 1;
                        for (int m = 0; m < k_r_ - 1; ++m) {
                            cum_mul = cum_mul * (i - m) * (j - m);
                        }
                        H(i, j) = cum_mul / (i+j-2*(k_r_+1));
                    } else {
                        H(i, j) = 0;
                    }
                }
            }
            double t0 = keyframes_[wp].getTime();
            double tend = keyframes_[wp + 1].getTime();
            H *= 1 / pow((tend - t0), 2*k_r_ - 1);
            H.transpose().colwise().reverse();
            H.transpose().colwise().reverse();
        }
    }
}
