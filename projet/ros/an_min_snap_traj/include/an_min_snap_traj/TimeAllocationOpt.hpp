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

#ifndef TIMEALLOCATIONOPT_H
#define TIMEALLOCATIONOPT_H

#include <alglib/optimization.h>
#include <nlopt.hpp>
#include "an_min_snap_traj/TrajectoryGenerator.hpp"

namespace an_min_snap_traj {
    class TimeAllocationOpt {
    public:
        enum Solver {ALGLIB, NLOPT_SLSQP, NLOPT_LBFGS, NLOPT_NEWTON, NLOPT_AUGLAG};
        typedef struct {
            TrajectoryGenerator* tg;
            TrajectoryGenerator::Solver solver;
            int iters;
        } generator_solver_pair_t;
        TimeAllocationOpt(TrajectoryGenerator *tg, TrajectoryGenerator::Solver solver);

        bool optimize(Solver s);

        static void timeAllocGrad(const alglib::real_1d_array &x, double &func,
                                    alglib::real_1d_array &grad, void *ptr);

        static void timeAllocFunc(const alglib::real_1d_array &x, double &func,
                                  void *ptr);

        static double nlopt_obj_func(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double nlopt_constr(const std::vector<double> &x, std::vector<double> &grad, void *data);
    private:
        TrajectoryGenerator* tg_;
        TrajectoryGenerator::Solver solver_;

        static alglib::real_1d_array eigen2alg(const Eigen::VectorXd vec);
        static alglib::real_2d_array eigen2alg(const Eigen::MatrixXd mat);
        static Eigen::VectorXd alg2eigen(const alglib::real_1d_array r);

        bool optimizeALGLIB();
        bool optimizeNLOPT(nlopt::algorithm alg);

        /**
         * Generate the gradient directions
         * @param m Number of SEGMENTS i.e. n_segments = n_keyframes - 1
         * @return
         */
        static Eigen::MatrixXd generateGi(const int m);
    };
}

#endif //TIMEALLOCATIONOPT_H
