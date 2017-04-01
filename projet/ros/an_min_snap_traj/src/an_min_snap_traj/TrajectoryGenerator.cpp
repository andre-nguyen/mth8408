#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "an_min_snap_traj/TrajectoryGenerator.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    TrajectoryGenerator::TrajectoryGenerator() {

    }

    void TrajectoryGenerator::addConstraint(TrajectoryConstraint tc) {
        if(getNumWaypoints() == 0) {
            assert(tc.getTime() == 0 );
            keyframes_.push_back(tc);
        } else {
            assert(tc.getTime() > keyframes_.back().getTime());
            keyframes_.push_back(tc);
        }
    }

    std::vector<TrajectoryConstraint> TrajectoryGenerator::getConstraints(){
        return keyframes_;
    }

    unsigned long TrajectoryGenerator::getNumWaypoints() {
        return keyframes_.size();
    }

    MatrixXd TrajectoryGenerator::getCostMatrix(int dim) const {
        return H_[dim];
    }

    int TrajectoryGenerator::getNumConstraints(int dim) const {
        int count = 0;
        for(std::vector<TrajectoryConstraint>::const_iterator it = keyframes_.cbegin();
                it != keyframes_.cend(); ++it){
            count += it->getConstraintCount(dim);
        }
        return count;
    }

    void TrajectoryGenerator::buildProblem() {
        for(int i = 0; i < states_; ++i) {
            buildCostMatrix(i);
            buildConstraintMatrix(i);
        }
    }

    void TrajectoryGenerator::buildCostMatrix(int dim) {
        unsigned long h_size = n_coeffs_ * (getNumWaypoints()-1);
        H_[dim] = MatrixXd::Zero(h_size, h_size);
        unsigned long wps = getNumWaypoints()-1;
        for (int wp = 0; wp < wps; ++wp) {
            MatrixXd H = MatrixXd::Zero(n_coeffs_, n_coeffs_);
            for (int i = 0; i <= n_; ++i) {
                for (int j = 0; j <= n_; ++j) {
                    if (i >= k_r_ && j >= k_r_) {
                        double cum_mul = 1;
                        for (int m = 0; m < k_r_; ++m) {
                            cum_mul = cum_mul * (i - m) * (j - m);
                        }
                        H(i, j) = cum_mul / (i+j-(2*k_r_)+1);
                    } else {
                        H(i, j) = 0;
                    }
                }
            }
            double t0 = keyframes_[wp].getTime();
            double tend = keyframes_[wp + 1].getTime();
            H *= 1 / pow((tend - t0), 2*k_r_ - 1);
            H.reverseInPlace(); // Equivalent to rot90(rot90(H)) in matlab
            // Block diagonal insertion
            /**
             * TODO check if I should be using a sparse matrix and if there is a difference
             * and if there is a difference in usage
             */
            unsigned long index = wp * n_coeffs_;
            H_[dim].block(index, index, n_coeffs_, n_coeffs_) = H;
#ifdef DEBUG
            std::cout << "intermediate cost matrix wp " << wp << " \n" << H << std::endl;
#endif
        }
    }

    void TrajectoryGenerator::buildConstraintMatrix(int dim) {
        std::vector<VectorXd> A;
        unsigned long wps = getNumWaypoints();
        int constraint_size = (wps-1) * n_coeffs_;
        for(int wp = 0; wp < wps; ++wp) {
            // We don't go to the snap because that's what we want to minimize I think...
            for(int der = Derivative::DER_POSITION; der < Derivative::DER_SNAP; ++der){
                // Don't add a constraint if it's unconstrained, duh!
                if(!keyframes_[wp].isConstrained(der, dim))
                    continue;

                if(wp == 0){
                    // Initial conditions, only add departure constraints
                    VectorXd a = VectorXd::Zero(constraint_size);
                    double t_next = keyframes_[wp+1].getTime();
                    double t_now = keyframes_[wp].getTime();
                    double int_t = 1 / std::pow(t_next - t_now, der);
                } else if(wp == wps-1) {
                    // Final conditions, only add arrival constraints
                    VectorXd a = VectorXd::Zero(constraint_size);
                    double t_now = keyframes_[wp].getTime();
                    double t_prev = keyframes_[wp-1].getTime();
                } else {
                    // Intermediate waypoint, add both departure and arrival constraints
                    VectorXd a1 = VectorXd::Zero(constraint_size);
                    VectorXd a2 = a1;
                    double t_next = keyframes_[wp+1].getTime();
                    double t_now = keyframes_[wp].getTime();
                    double t_prev = keyframes_[wp-1].getTime();
                }
            }
        }
    }
}
