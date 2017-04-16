//
// Created by andrephu-vannguyen on 15/04/17.
//
#include <algorithm>
#include <iostream>
#include "an_min_snap_traj/TrajectoryGenerator.hpp"
#include "an_min_snap_traj/TimeAllocationOpt.hpp"

namespace an_min_snap_traj {
    TimeAllocationOpt::TimeAllocationOpt(TrajectoryGenerator *tg) : tg_(tg){}

    bool TimeAllocationOpt::optimize() {
        /**
         * Where f(x) is the value of the objective function used
         * for trajectory generation, we want solve
         *      min f(x)
         *      s.t. sum(Ti) = t_final
         *           Ti >= 0
         * Where Ti are the segment times from waypoint t_i-1 to t_i
         *
         * To translate to the alglib formulation we have
         * x -> segment times
         * c -> 1 row all ones = 0
         * li -> lower bound constraint = 0
         */
        alglib::real_1d_array x = eigen2alg(tg_->getArrivalTimes());
        double ones[tg_->getNumWaypoints()];
        std::fill_n(ones, tg_->getNumWaypoints(), 1.0);
        alglib::real_2d_array c;
        c.setcontent(1, tg_->getNumWaypoints(), ones);
        alglib::integer_1d_array ct = "[0]";
        alglib::minbleicstate state;
        alglib::minbleicreport rep;

        double epsg = 0.000001;
        double epsf = 0;
        double epsx = 0;
        alglib::ae_int_t maxits = 0;
        minbleiccreate(x, state);
        minbleicsetlc(state, c, ct);
        minbleicsetcond(state, epsg, epsf, epsx, maxits);
        alglib::minbleicoptimize(state, TimeAllocationOpt::timeAllocGrad, NULL, (void*)tg_);
        minbleicresults(state, x, rep);
        std::cout << "term type " << rep.terminationtype;
        return false;
    }

    void TimeAllocationOpt::timeAllocGrad(const alglib::real_1d_array &x, double &func,
                              alglib::real_1d_array &grad, void *ptr) {
        const double h_ = 0.001;
        TrajectoryGenerator* tg = (TrajectoryGenerator*)ptr;

        // Evaluate trajectory with current time distribution
        VectorXd arrival_times = segment2time(alg2eigen(x));
        tg->setArrivalTimes(arrival_times);
        tg->solveProblem(TrajectoryGenerator::Solver::OOQP);
        func = tg->getObjectiveFuncVal();

        // Evaluate gradient
        long m = tg->getNumWaypoints() - 1; // Match matlab code :-)
        MatrixXd gi = generateGi(m);
        grad.setlength(m);
        for(int i = 0; i < m; ++i) {
            VectorXd t = segtimeRealloc(arrival_times, h_ * gi.col(i));
            tg->setArrivalTimes(t);
            tg->solveProblem(TrajectoryGenerator::Solver::OOQP);
            grad(i) = (tg->getObjectiveFuncVal() - func) / h_;
        }

    }

    alglib::real_1d_array TimeAllocationOpt::eigen2alg(const Eigen::VectorXd vec) {
        alglib::real_1d_array r;
        r.setcontent(vec.size(), vec.data());
        return r;
    }

    alglib::real_2d_array TimeAllocationOpt::eigen2alg(const Eigen::MatrixXd mat) {
        alglib::real_2d_array r;
        r.setlength(mat.rows(), mat.cols());
        for(int i = 0; i < mat.rows(); ++i) {
            for(int j = 0; j < mat.cols(); ++j) {
                r(i,j) = mat(i,j);
            }
        }
        return r;
    }

    Eigen::VectorXd TimeAllocationOpt::alg2eigen(const alglib::real_1d_array r) {

    }

    Eigen::MatrixXd TimeAllocationOpt::generateGi(const int m) {
        MatrixXd mat(m,m);
        mat.fill(-1.0/(m-1));
        for(int i = 0; i < m; ++i)
            mat(i,i) = 1;
        return mat;
    }
}