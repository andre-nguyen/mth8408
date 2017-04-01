#ifndef GEN_TRAJ_CPP
#define GEN_TRAJ_CPP

#include <vector>
#include <Eigen/Core>
#include "an_min_snap_traj/TrajectoryConstraint.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    class TrajectoryGenerator {
    public:
        TrajectoryGenerator();

        /**
         * Add a constraint to the trajectory
         * @param tc The constraint
         */
        void addConstraint(TrajectoryConstraint tc);

        std::vector<TrajectoryConstraint> getConstraints();

        /**
         * Get the number of waypoints (constraints) in the trajectory
         * includes the initial conditions
         * @return unsigned long waypoints
         */
        unsigned long getNumWaypoints();

        /**
         * Get the cost matrix for a certain dimension of the QP problem
         * @param dim
         * @return
         */
        MatrixXd getCostMatrix(int dim) const;

        int getNumConstraints(int dim) const;

        /**
         * Build the required matrices for the QP problem using the
         * constraints currently added to the generator.
         */
        void buildProblem();

    private:
        const int k_r_ = 4;   // Order of the derivative of the position
        const int n_ = 6;     // Order of the polynomials describing the trajectory
        const int n_coeffs_ = n_ + 1; // Number of coefficients in a polynomial
        const int states_ = 3;// Number of states in the problem
        std::vector<TrajectoryConstraint> keyframes_;   // Constraints on the trajectory
                                                        // including initial conditions
        MatrixXd H_[3]; // Cost matrices

        const int X = 0;
        const int Y = 1;
        const int Z = 2;

        /**
         * Build the cost matrix for a certain dimension dim
         * and store it in H_. Follows the equations outlined in
         * Adam Bry's thesis and richter_rss13_workshop
         * @param dim
         */
        void buildCostMatrix(int dim);

        /**
         * Build the constraint matrix for the QP problem
         * any constraints marked with std::numeric_limits<double>::max()
         * will be treated as unconstrained
         * @param dim
         */
        void buildConstraintMatrix(int dim);
    };
}

#endif
