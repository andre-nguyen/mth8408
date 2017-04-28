#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include <ooqp_eigen_interface/OoqpEigenInterface.hpp>
#include <eigen-quadprog/QuadProg.h>
#ifdef USE_GUROBI
#include "gurobi_c++.h"
#include <eigen-gurobi/Gurobi.h>
#endif
#ifdef USE_QPOASES
#include <qpOASES/QProblem.hpp>
#endif

#include <an_min_snap_traj/TrajectorySegment.hpp>
#include "an_min_snap_traj/TrajectoryGenerator.hpp"
#include <an_min_snap_traj/IpoptAdapter.h>
#include <an_min_snap_traj/QuadraticProgramming.h>
#include <an_min_snap_traj/IpoptSolver.h>
#include "an_min_snap_traj/IpoptProblem.h"

using namespace Eigen;

namespace an_min_snap_traj {
    TrajectoryGenerator::TrajectoryGenerator() {
        problemBuilt_ = false;
        isDiscretized_ = false;
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

    std::vector<Vector3d> TrajectoryGenerator::discretizeSolution() {
        std::vector<Vector3d> fulltraj;
        for(auto segment = solutionSegments_.begin(); segment != solutionSegments_.end();
            ++segment) {
            auto seg_traj = segment->discretize(dt_);
            fulltraj.insert(fulltraj.end(),
                            seg_traj.begin(),
                            seg_traj.end());
        }
        isDiscretized_ = true;
        return fulltraj;
    }

    VectorXd TrajectoryGenerator::getArrivalTimes() const {
        VectorXd times(keyframes_.size());
        for(int i = 0; i < keyframes_.size(); ++i){
            times(i) = keyframes_[i].getTime();
        }
        return times;
    }

    double TrajectoryGenerator::getObjectiveFuncVal() const {
        double val = 0;
        for(int i = 0; i < STATE_COUNT; ++i) {
            val += solution_[i].transpose() * H_[i] * solution_[i];
        }
        return val;
    }

    std::vector<Vector3d> TrajectoryGenerator::getDiscreteSolution(Derivative der) {
        std::vector<Vector3d> res;
        if(!isDiscretized_)
            return res;

        for(auto segment = solutionSegments_.begin(); segment != solutionSegments_.end();
                ++segment) {
            auto seg_traj = segment->getTraj(der);
            res.insert(res.end(),
                        seg_traj.begin(),
                        seg_traj.end());
        }
        return res;
    }

    void TrajectoryGenerator::setArrivalTimes(VectorXd times) {
        assert(times.size() == keyframes_.size());
        assert(times(0) == 0);
        for(int i = 0; i < times.size(); ++i) {
            keyframes_[i].setTime(times(i));
        }
        problemBuilt_ = false;
        isDiscretized_ = false;
    }

    void TrajectoryGenerator::buildProblem() {
        for(int i = 0; i < states_; ++i) {
            buildCostMatrix(i);
            buildConstraintMatrix(i);
        }
        problemBuilt_ = true;
    }

    bool TrajectoryGenerator::solveProblem(Solver solver) {
        bool result = true;
        if(!problemBuilt_)
            buildProblem();

        VectorXd poly[State::STATE_COUNT][getNumWaypoints()-1];
        for(int i = 0; i < State::STATE_COUNT; ++i){
            result &= solveProblem(i, solver);
            if(result) {
                // slice up the solution vector
                for(int wp = 0; wp < getNumWaypoints()-1; ++wp) {
                    poly[i][wp] = solution_[i].segment(wp*n_coeffs_, n_coeffs_);
                }
            }
        }

        // Got all the solutions, now create trajectory segments
        // YOLO alpha is 1
        for(int wp = 0; wp < getNumWaypoints()-1; ++wp) {
            TrajectorySegment ts(keyframes_[wp].getTime(), keyframes_[wp+1].getTime(), 1,
                                 poly[0][wp], poly[1][wp], poly[2][wp]);
            solutionSegments_.push_back(ts);
        }

        return result;
    }

    bool TrajectoryGenerator::solveProblem(int dim, Solver solver) {
        if(!problemBuilt_)
            return false;

        bool result = false;
        switch(solver) {
            case Solver::OOQP:
                result = solveProblemOoqp(dim);
                break;
            case Solver::GUROBI:
                result =  solveProblemGurobi(dim);
                break;
            case Solver::QLD:
                result =  solveProblemQld(dim);
                break;
            case Solver::QUADPROG:
                result =  solveProblemQuadprog(dim);
                break;
            case Solver::IPOPT:
                result = solveProblemIPOPT(dim);
                break;
            case Solver::QPOASES:
                result = solveProblemqpOASES(dim);
            default:
                result = false;
                break;
        }

        return result;
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
        ooqpei::OoqpEigenInterface::setIsInDebugMode(true);
        return ooqpei::OoqpEigenInterface::solve(Q, c, A, b, C, d, f, solution_[dim]);
    }

    bool TrajectoryGenerator::solveProblemGurobi(int dim) {
        bool success = false;
#ifdef USE_GUROBI
        int n_fixed_constr = A_fixed_[dim].rows();
        int n_cont_constr = A_continuity_[dim].rows();
        int n_vars = H_[dim].cols();
        std::cout << "DEBUG DBUG " << n_fixed_constr << std::endl
                  << n_cont_constr << std::endl
                  << n_vars << std::endl;
        VectorXd C = VectorXd::Zero(n_vars);
        MatrixXd Aeq(n_fixed_constr + n_cont_constr, n_vars);
        Aeq << A_fixed_[dim], A_continuity_[dim];
        VectorXd Beq(n_fixed_constr + n_cont_constr);
        Beq << b_fixed_[dim], b_continuity_[dim];
        MatrixXd Aineq;
        VectorXd Bineq;
        VectorXd XL(n_vars), XU(n_vars);
        XL.fill(-std::numeric_limits<double>::max());
        XU.fill(std::numeric_limits<double>::max());
        GurobiDense qp(n_vars, n_fixed_constr + n_cont_constr, 0);
        MatrixXd Q = H_[dim] + (1e-2 * MatrixXd::Identity(H_[dim].rows(), H_[dim].cols()));
        qp.solve(Q, C, Aeq, Beq, Aineq, Bineq, XL, XU);
        std::cout << qp.result() << std::endl;
#endif
        return success;
    }

    bool TrajectoryGenerator::solveProblemQld(int dim) {
        return false;
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
        return false;
    }

    bool TrajectoryGenerator::solveProblemIPOPT(int dim) {
        QuadraticProgramming* qp = new QuadraticProgramming(H_[dim].cols(), b_fixed_[dim].rows() + b_continuity_[dim].rows());
        SparseMatrix<double> h_sparse = H_[dim].sparseView();
        std::vector<Triplet<double>> sparse_triplets;
        for(int k = 0; k < h_sparse.outerSize(); ++k){
            for(SparseMatrix<double>::InnerIterator it(h_sparse, k); it; ++it) {
                Triplet<double> t(it.row(), it.col(), it.value());
                sparse_triplets.push_back(t);
            }
        }
        qp->SetH(sparse_triplets);

        int n_fixed_constr = A_fixed_[dim].rows();
        int n_cont_constr = A_continuity_[dim].rows();
        int n_vars = H_[dim].cols();
        MatrixXd Aeq(n_fixed_constr + n_cont_constr, n_vars);
        Aeq << A_fixed_[dim], A_continuity_[dim];
        SparseMatrix<double> Aeq_sparse = Aeq.sparseView();
        std::vector<Triplet<double>> a_sparse_triplets;
        for(int k = 0; k < Aeq_sparse.outerSize(); ++k){
            for(SparseMatrix<double>::InnerIterator it(Aeq_sparse, k); it; ++it) {
                Triplet<double> t(it.row(), it.col(), it.value());
                a_sparse_triplets.push_back(t);
            }
        }

        VectorXd Beq(n_fixed_constr + n_cont_constr);
        Beq << b_fixed_[dim], b_continuity_[dim];
        qp->SetA(a_sparse_triplets);
        qp->Setb(Beq);

        VectorXd c = VectorXd::Zero(n_vars);
        qp->Setc(c);

        VectorXd lb(n_vars), ub(n_vars);
        lb.fill(-std::numeric_limits<double>::max());
        ub.fill(std::numeric_limits<double>::max());
        qp->SetLowerBoundary(lb);
        qp->SetUpperBoundary(ub);

        IpoptSolver solver(qp);
        solver.Solve();
        std::cout << qp->GetOptimalSolution();
        solution_[dim] =qp->GetOptimalSolution();
        return true;
    }

    bool TrajectoryGenerator::solveProblemqpOASES(int dim) {
#ifdef USE_QPOASES
        /**
         *  min     1/2*x'Hx + x'g
         *  s.t.    lb  <=  x <= ub
         *          lbA <= Ax <= ubA
         */
        int n_constr = getConstraintVector(dim).size();
        int n_vars = H_[dim].cols();
        qpOASES::QProblem qp(n_vars, n_constr);
        qpOASES::Options opt;
        opt.enableEqualities = qpOASES::BT_TRUE;    // Super important or else opt will fail
        qp.setOptions(opt);


        double H_arr[n_vars * n_vars];
        eigenMat2buf(H_[dim], H_arr);
        VectorXd g = VectorXd::Zero(getCostMatrix(dim).rows()); // zero linear term
        double A_arr[n_constr * n_vars];
        eigenMat2buf(getConstraintMatrix(dim), A_arr);
        int nWSR = 100;
        //qp.printOptions();
        //qp.printProperties();
        qp.setPrintLevel(qpOASES::PrintLevel::PL_NONE);
        //std::cout << "DEBUG " << g;
        qp.init(H_arr,
                g.data(),
                A_arr,
                NULL,
                NULL,
                getConstraintVector(dim).data(),
                getConstraintVector(dim).data(),
                nWSR,
                NULL
        );
        double x[n_vars];
        qp.getPrimalSolution(x);
        solution_[dim] = VectorXd(n_vars);
        for(int i = 0; i < n_vars; i++)
            solution_[dim](i) = x[i];
#endif
        return true;
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
