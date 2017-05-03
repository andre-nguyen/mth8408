/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2017 Mobile Robotics and Autonomous Systems Laboratory (MRASL),
 * Polytechnique Montreal. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/
#include <algorithm>
#include <iostream>
#include "an_min_snap_traj/TrajectoryGenerator.hpp"
#include "an_min_snap_traj/TimeAllocationOpt.hpp"

namespace an_min_snap_traj {
    TimeAllocationOpt::TimeAllocationOpt(TrajectoryGenerator *tg,
        TrajectoryGenerator::Solver solver) : tg_(tg){}

    bool TimeAllocationOpt::optimize(Solver s) {
        bool result = false;
        switch (s) {
            case Solver::ALGLIB:
                result = optimizeALGLIB();
                break;
            case Solver::NLOPT_SLSQP:
                result = optimizeNLOPT(nlopt::algorithm::LD_SLSQP);
                break;
            case Solver::NLOPT_LBFGS:
                result = optimizeNLOPT(nlopt::algorithm::LD_LBFGS);
                break;
            case Solver::NLOPT_NEWTON:
                result = optimizeNLOPT(nlopt::algorithm::LD_TNEWTON_PRECOND);
                break;
            case Solver::NLOPT_AUGLAG:
                result = optimizeNLOPT(nlopt::algorithm::AUGLAG_EQ);
                break;
            default:
                result = false;
                break;
        }
        return result;
    }

    bool TimeAllocationOpt::optimizeALGLIB() {
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
        for(int j = 0; j < mat.cols(); ++j) {
            for(int i = 0; i < mat.rows(); ++i) {
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

    bool TimeAllocationOpt::optimizeNLOPT(nlopt::algorithm alg){
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
        auto segment_times = time2segment(times);
        nlopt::opt opt(nlopt::algorithm::LD_AUGLAG_EQ, segment_times.rows());
        generator_solver_pair_t* gsp = new generator_solver_pair_t();
        gsp->tg = tg_;
        gsp->solver = solver_;
        gsp->iters = 0;
        opt.set_min_objective(nlopt_obj_func, (void*)gsp);

        // 2. Set linear constraints, sum of segment times = last arrival time
        opt.set_lower_bounds(0);
        double last = times(times.size()-1);
        opt.add_equality_constraint(nlopt_constr, (void*)&last, 1e-8);
        //double lb[segment_times.rows()];
        //memset(lb, 0, segment_times.rows());


        // 3. Set bounds constraints lc = 0 uc = inf

        // 4. Set optimization termination conditions
        opt.set_xtol_rel(1e-8);
        opt.set_maxeval(2);

        // 5. Optimize!
        std::vector<double> t(segment_times.rows());
        for(auto it = t.begin(); it != t.end(); ++it)
            *it = 1;
        double minf;
        nlopt::result res = opt.optimize(t, minf);
        switch(res) {
            case nlopt::SUCCESS:
                std::cout << "success" << std::endl;
                break;
            case nlopt::STOPVAL_REACHED:
                std::cout << "STOPVAL_REACHED" << std::endl;
                break;
            case nlopt::FTOL_REACHED:
                std::cout << "FTOL_REACHED" << std::endl;
                break;
            case nlopt::XTOL_REACHED:
                std::cout << "XTOL_REACHED" << std::endl;
                break;
            case nlopt::MAXEVAL_REACHED:
                std::cout << "MAXEVAL_REACHED" << std::endl;
                break;
            case nlopt::MAXTIME_REACHED:
                std::cout << "MAXTIME_REACHED" << std::endl;
                break;
            default:
                std::cout << "failed" << std::endl;
                break;
        }
        std::cout << "result " << (res > 0 ? "worked!\n" : "failed\n");
        for(auto it = t.begin(); it != t.end(); ++it)
            std::cout << *it << ", ";
        std::cout << "\nminf " << minf << std::endl;
        std::cout << "iters " << gsp->iters << std::endl;
        return false;
    }

    double TimeAllocationOpt::nlopt_obj_func(const std::vector<double> &x,
                                           std::vector<double> &grad, void* f_data) {
        const double h_ = 0.001;
        generator_solver_pair_t* gsp = (generator_solver_pair_t*)f_data;
        auto tg = gsp->tg;
        auto solver = gsp->solver;
        gsp->iters += 1;

        //Evaluate trajectory with current time distribution
        VectorXd times(x.size());
        for(int i = 0; i < x.size(); ++i) {
            times(i) = x[i];
        }
        VectorXd arrival_times = segment2time(times);
        tg->setArrivalTimes(arrival_times);
        tg->solveProblem(solver);
        auto value = tg->getObjectiveFuncVal();

        // Evaluate gradient
        if(!grad.empty()) {
            long m = tg->getNumWaypoints() - 1; // Match matlab code :-)
            MatrixXd gi = generateGi(m);
            for (int i = 0; i < m; ++i) {
                VectorXd t = segtimeRealloc(arrival_times, h_ * gi.col(i));
                tg->setArrivalTimes(t);
                tg->solveProblem(solver);
                grad[i] = (tg->getObjectiveFuncVal() - value) / h_;
            }
        }

        return value;
    }

    double TimeAllocationOpt::nlopt_constr(const std::vector<double> &x, std::vector<double> &grad,
                                           void *data) {
        double last = *((double*)data);
        if(!grad.empty()){
            for(auto it = grad.begin(); it != grad.end(); ++it)
                *it = 1;
        }

        double sum = 0.0;
        for(auto it = x.begin(); it != x.end(); ++it) {
            sum += *it;
        }
        return sum - last;
    }
}