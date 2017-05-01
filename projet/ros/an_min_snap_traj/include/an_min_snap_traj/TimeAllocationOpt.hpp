//
// Created by andrephu-vannguyen on 15/04/17.
//

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
