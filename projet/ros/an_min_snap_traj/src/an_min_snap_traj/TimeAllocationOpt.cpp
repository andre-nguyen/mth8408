//
// Created by andrephu-vannguyen on 15/04/17.
//
#include <algorithm>
#include <iostream>
#include "an_min_snap_traj/TrajectoryGenerator.hpp"
#include "an_min_snap_traj/TimeAllocationOpt.hpp"

namespace an_min_snap_traj {
    TimeAllocationOpt::TimeAllocationOpt(TrajectoryGenerator *tg,
        TrajectoryGenerator::Solver solver) : tg_(tg){}

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
        // 1. Create problem
        auto times = tg_->getArrivalTimes();
        alglib::real_1d_array x = eigen2alg(time2segment(times));
        alglib::minbleicstate state;
        alglib::minbleiccreate(x, state);

        // 2. Set linear constraints, sum of segment times = last arrival time
        double ones[x.length() + 1];
        std::fill_n(ones, x.length(), 1.0);
        ones[x.length()] = times(times.size()-1);   // Last element is the rhs
        alglib::real_2d_array c;
        c.setcontent(1, x.length()+1, ones);

        alglib::integer_1d_array ct = "[0]";  // equality constraint
        alglib::minbleicsetlc(state, c, ct);

        // 3. Set bounds constraints lc = 0 uc = inf
        //double zeros[x.length()]


        // 4. Set optimization termination conditions
        double epsg = 1e-6;
        double epsf = 0;
        double epsx = 0;
        alglib::ae_int_t maxits = 0;
        alglib::minbleicsetcond(state, epsg, epsf, epsx, maxits);

        // 5. Optimize!
        generator_solver_pair_t* gsp = new generator_solver_pair_t();
        gsp->tg = tg_;
        gsp->solver = solver_;
        alglib::minbleicoptimize(state, TimeAllocationOpt::timeAllocGrad, NULL, (void*)gsp);

        // 6. Get results
        alglib::minbleicreport rep;
        alglib::minbleicresults(state, x, rep);
        std::cout << "term type " << (int)rep.terminationtype << std::endl;
        std::cout << "iter cnt  " << (int)rep.iterationscount << std::endl;
        std::cout << "solution\n" << alg2eigen(x) << std::endl;
        std::cout << "solution times\n" << segment2time(alg2eigen(x)) << std::endl;

        return (rep.terminationtype > 0);
    }

    void TimeAllocationOpt::timeAllocGrad(const alglib::real_1d_array &x, double &func,
                              alglib::real_1d_array &grad, void *ptr) {
        const double h_ = 0.001;
        generator_solver_pair_t* gsp = (generator_solver_pair_t*)ptr;
        auto tg = gsp->tg;
        auto solver = gsp->solver;

        // Evaluate trajectory with current time distribution
        //std::cout << "alg2eig \n" << alg2eigen(x) << std::endl;
        //std::cout << "seg2time \n" << segment2time(alg2eigen(x)) << std::endl;
        VectorXd arrival_times = segment2time(alg2eigen(x));
        tg->setArrivalTimes(arrival_times);
        tg->solveProblem(solver);
        func = tg->getObjectiveFuncVal();

        // Evaluate gradient
        long m = tg->getNumWaypoints() - 1; // Match matlab code :-)
        MatrixXd gi = generateGi(m);
        alglib::real_1d_array nabla;
        nabla.setlength(m);
        for(int i = 0; i < m; ++i) {
            VectorXd t = segtimeRealloc(arrival_times, h_ * gi.col(i));
            tg->setArrivalTimes(t);
            tg->solveProblem(solver);
            nabla(i) = (tg->getObjectiveFuncVal() - func) / h_;
        }
        grad = nabla;
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
        Eigen::VectorXd v(r.length());
        for(int i = 0; i < r.length(); ++i)
            v(i) = r(i);
        return v;
    }

    Eigen::MatrixXd TimeAllocationOpt::generateGi(const int m) {
        MatrixXd mat(m,m);
        mat.fill(-1.0/(m-1));
        for(int i = 0; i < m; ++i)
            mat(i,i) = 1;
        return mat;
    }
}