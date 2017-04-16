//
// Created by andrephu-vannguyen on 15/04/17.
//

#ifndef TIMEALLOCATIONOPT_H
#define TIMEALLOCATIONOPT_H

#include <alglib/optimization.h>
#include "an_min_snap_traj/TrajectoryGenerator.hpp"

namespace an_min_snap_traj {
    class TimeAllocationOpt {
    public:
        typedef struct {
            TrajectoryGenerator* tg;
            TrajectoryGenerator::Solver solver;
        } generator_solver_pair_t;
        TimeAllocationOpt(TrajectoryGenerator *tg, TrajectoryGenerator::Solver solver);

        bool optimize();

        static void timeAllocGrad(const alglib::real_1d_array &x, double &func,
                                    alglib::real_1d_array &grad, void *ptr);

        static void timeAllocFunc(const alglib::real_1d_array &x, double &func,
                                  void *ptr);
    private:
        TrajectoryGenerator* tg_;
        TrajectoryGenerator::Solver solver_;

        static alglib::real_1d_array eigen2alg(const Eigen::VectorXd vec);
        static alglib::real_2d_array eigen2alg(const Eigen::MatrixXd mat);
        static Eigen::VectorXd alg2eigen(const alglib::real_1d_array r);

        /**
         * Generate the gradient directions
         * @param m Number of SEGMENTS i.e. n_segments = n_keyframes - 1
         * @return
         */
        static Eigen::MatrixXd generateGi(const int m);
    };
}

#endif //TIMEALLOCATIONOPT_H
