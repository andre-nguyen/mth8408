#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <ooqp_eigen_interface/OoqpEigenInterface.hpp>
#include <eigen-quadprog/QuadProg.h>
#include "an_min_snap_traj/TrajectoryGenerator.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    TrajectoryGenerator::TrajectoryGenerator() {
        problemBuilt_ = false;
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

    MatrixXd TrajectoryGenerator::getConstraintMatrix(int dim) const {
        MatrixXd A_temp(A_fixed_[dim].rows() + A_continuity_[dim].rows(), A_fixed_[dim].cols());
        A_temp << A_fixed_[dim] , A_continuity_[dim];
        return A_temp;
    }

    VectorXd TrajectoryGenerator::getConstraintVector(int dim) const {
        VectorXd b(b_fixed_[dim].size() + b_continuity_[dim].size());
        b << b_fixed_[dim] , b_continuity_[dim];
        return b;
    }

    MatrixXd TrajectoryGenerator::getFixedConstraintMatrix(int dim) const {
        return A_fixed_[dim];
    }

    VectorXd TrajectoryGenerator::getFixedConstraintVector(int dim) const {
        return b_fixed_[dim];
    }

    MatrixXd TrajectoryGenerator::getContinuityConstraintMatrix(int dim) const {
        return A_continuity_[dim];
    }

    VectorXd TrajectoryGenerator::getContinuityConstraintVector(int dim) const {
        return b_continuity_[dim];
    }

    int TrajectoryGenerator::getNumConstraints(int dim) const {
        int count = 0;
        for(std::vector<TrajectoryConstraint>::const_iterator it = keyframes_.cbegin();
                it != keyframes_.cend(); ++it){
            count += it->getConstraintCount(dim);
        }
        return count;
    }

    VectorXd TrajectoryGenerator::getSolution(int dim) const {
        return solution_[dim];
    }

    void TrajectoryGenerator::buildProblem() {
        for(int i = 0; i < states_; ++i) {
            buildCostMatrix(i);
            buildConstraintMatrix(i);
        }
        problemBuilt_ = true;
    }

    bool TrajectoryGenerator::solveProblem(int dim, Solver solver) {
        if(!problemBuilt_)
            return false;

        switch(solver) {
            case Solver::OOQP:
                return solveProblemOoqp(dim);
            case Solver::GUROBI:
                return solveProblemGurobi(dim);
            case Solver::QLD:
                return solveProblemQld(dim);
            case Solver::QUADPROG:
                return solveProblemQuadprog(dim);
            default:
                return false;
        }
    }

    bool TrajectoryGenerator::solveProblemOoqp(int dim) {
        /**
         *  Don't use ooqpei's quadratic problem formulation module,
         *  the way we developed the equations can be directly sent to
         *  the OoqpEigenInterface::solve function.
         *  Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and l <= x <= u
         */
        SparseMatrix<double, RowMajor> Q = getCostMatrix(dim).sparseView();
        // We have no linear term
        VectorXd c = VectorXd::Zero(getCostMatrix(dim).rows());
        SparseMatrix<double, RowMajor> A = getConstraintMatrix(dim).sparseView();
        auto b = getConstraintVector(dim);
        // Empty vectors and matrices for the rest of the params
        Eigen::SparseMatrix<double, Eigen::RowMajor> C;
        Eigen::VectorXd d, f;
        //ooqpei::OoqpEigenInterface::setIsInDebugMode(true);
        return ooqpei::OoqpEigenInterface::solve(Q, c, A, b, C, d, f, solution_[dim]);
    }

    bool TrajectoryGenerator::solveProblemGurobi(int dim) {

    }

    bool TrajectoryGenerator::solveProblemQld(int dim) {

    }

    bool TrajectoryGenerator::solveProblemQuadprog(int dim) {
        Eigen::QuadProgDense qp(H_[dim].rows(), b_fixed_[dim].rows() + b_continuity_[dim].rows(),
            0);
        VectorXd c = VectorXd::Zero(getCostMatrix(dim).rows()); // zero linear term
        auto Aeq = getConstraintMatrix(dim);
        auto beq = getConstraintVector(dim);
        MatrixXd Aineq;
        VectorXd bineq;
        qp.solve(H_[dim], c, Aeq, beq, Aineq, bineq);
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
            H = rot90(H, 2); // Equivalent to rot90(rot90(H)) in matlab
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
        std::vector<VectorXd> A_eq;
        std::vector<double> b_eq;
        unsigned long wps = getNumWaypoints();
        int constraint_size = (wps-1) * n_coeffs_;
        MatrixXd I = rot90(MatrixXd::Identity(n_coeffs_, n_coeffs_));
        MatrixXd coeffs = genCoefficientMatrix(n_, k_r_);
        for(int wp = 0; wp < wps; ++wp) {
            // We don't go to the snap because that's what we want to minimize I think...
            for(int der = Derivative::DER_POSITION; der < Derivative::DER_SNAP; ++der){
                // Don't add a constraint if it's unconstrained, duh!
                if(!keyframes_[wp].isConstrained(der, dim))
                    continue;

                if(wp == 0){
                    // Initial conditions, only add departure constraints
                    double t_next = keyframes_[wp+1].getTime();
                    double t_now = keyframes_[wp].getTime();
                    double int_t = 1 / std::pow(t_next - t_now, der);

                    VectorXd polynomial = coeffs.row(der).cwiseProduct(I.row(der)) * int_t;
                    unsigned int idx = wp * n_coeffs_;
                    VectorXd a = VectorXd::Zero(constraint_size);
                    a.segment(idx, n_coeffs_) << polynomial;
                    double b = keyframes_[wp].getConstraint(der)(dim);
                    A_eq.push_back(a);
                    b_eq.push_back(b);

                } else if(wp == wps-1) {
                    // Final conditions, only add arrival constraints
                    double t_now = keyframes_[wp].getTime();
                    double t_prev = keyframes_[wp-1].getTime();
                    double int_t_prev = 1 / std::pow(t_now - t_prev, der);

                    VectorXd polynomial = coeffs.row(der) * int_t_prev;
                    unsigned int idx = (wp-1) * n_coeffs_;
                    VectorXd a = VectorXd::Zero(constraint_size);
                    a.segment(idx, n_coeffs_) << polynomial;
                    double b = keyframes_[wp].getConstraint(der)(dim);
                    A_eq.push_back(a);
                    b_eq.push_back(b);
                } else {
                    // Intermediate waypoint, add both departure and arrival constraints
                    VectorXd a1 = VectorXd::Zero(constraint_size);
                    VectorXd a2 = a1;
                    double t_next = keyframes_[wp+1].getTime();
                    double t_now = keyframes_[wp].getTime();
                    double t_prev = keyframes_[wp-1].getTime();

                    // Arrival constraint
                    double int_t_prev = 1 / std::pow(t_now - t_prev, der);
                    VectorXd polynomial = coeffs.row(der) * int_t_prev;
                    unsigned int idx = (wp-1) * n_coeffs_;
                    a1.segment(idx, n_coeffs_) << polynomial;

                    // Departure constraint
                    double int_t = 1 / std::pow(t_next - t_now, der);
                    polynomial = coeffs.row(der).cwiseProduct(I.row(der)) * int_t_prev;
                    idx = wp * n_coeffs_;
                    a2.segment(idx, n_coeffs_) << polynomial;

                    A_eq.push_back(a1);
                    A_eq.push_back(a2);
                    double b = keyframes_[wp].getConstraint(der)(dim);
                    b_eq.push_back(b);  // !!Both a1 and a2 are equal to the same thing
                    b_eq.push_back(b);
                }
            }
        }
        // Dump the vectors into the Eigen matrices
        assert(A_eq.size() == b_eq.size()); //Would be pretty awks if this wasn't true
        A_fixed_[dim] = MatrixXd(A_eq.size(), constraint_size);
        b_fixed_[dim] = VectorXd(b_eq.size());

        for(int i = 0; i < A_eq.size(); ++i){
            A_fixed_[dim].row(i) = A_eq[i];
            b_fixed_[dim](i) = b_eq[i];
        }

        A_eq.clear();
        b_eq.clear();

        /**
         * Do the continuity constraints
         * Basically for every unconstrained derivative, make the first
         * polynomial segment equal to the second polynomial segment
         */
        // Purposely skip first and last waypoint, only want intermediate waypoints
        for(int wp = 1; wp < wps-1; ++wp){
            for(int der = Derivative::DER_POSITION; der < Derivative::DER_COUNT; ++der){
                // If the derivative was constrained, we already added it previously
                if(keyframes_[wp].isConstrained(der, dim))
                    continue;
                double t_next = keyframes_[wp+1].getTime();
                double t_now = keyframes_[wp].getTime();
                double t_prev = keyframes_[wp-1].getTime();
                double int_t_prev = 1 / std::pow(t_now - t_prev, der);
                double int_t = 1 / std::pow(t_next - t_now, der);
                VectorXd a = VectorXd::Zero(constraint_size);

                // From prev wp to now
                VectorXd polynomial = coeffs.row(der) * int_t_prev;
                unsigned int idx = (wp-1) * n_coeffs_;
                a.segment(idx, n_coeffs_) << polynomial;

                // from now to next wp
                polynomial = coeffs.row(der).cwiseProduct(I.row(der)) * (-int_t);
                idx = wp * n_coeffs_;
                a.segment(idx, n_coeffs_) << polynomial;
                A_eq.push_back(a);
                b_eq.push_back(0);
            }
        }

        // Dump the vectors into the eigen matrices
        assert(A_eq.size() == b_eq.size()); //Would be pretty awks if this wasn't true
        A_continuity_[dim] = MatrixXd(A_eq.size(), constraint_size);
        b_continuity_[dim] = VectorXd(b_eq.size());

        for(int i = 0; i < A_eq.size(); ++i) {
            A_continuity_[dim].row(i) = A_eq[i];
            b_continuity_[dim](i) = b_eq[i];
        }

    }
}
