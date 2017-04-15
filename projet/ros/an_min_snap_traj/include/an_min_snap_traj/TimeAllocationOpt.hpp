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
        TimeAllocationOpt(TrajectoryGenerator *tg);

        bool optimize();

        static void timeAllocGrad(const alglib::real_1d_array &x, double &func,
                                    alglib::real_1d_array &grad, void *ptr);
    private:
        TrajectoryGenerator* tg_;

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
