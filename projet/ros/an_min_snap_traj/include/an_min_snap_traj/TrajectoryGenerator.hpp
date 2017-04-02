#ifndef GEN_TRAJ_CPP
#define GEN_TRAJ_CPP

#include <vector>
#include <Eigen/Core>
#include "an_min_snap_traj/TrajectoryConstraint.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    class TrajectoryGenerator {
    public:
        enum Solver {OOQP, GUROBI, QLD};

        TrajectoryGenerator();

        /**
         * Add a constraint to the trajectory
         * @param tc The constraint
         */
        void addConstraint(TrajectoryConstraint tc);

        /**
         * Get a copy of the vector of constraints
         * @return
         */
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

        MatrixXd getFixedConstraintMatrix(int dim) const;
        VectorXd getFixedConstraintVector(int dim) const;

        MatrixXd getContinuityConstraintMatrix(int dim) const;
        VectorXd getContinuityConstraintVector(int dim) const;

        VectorXd getSolution(int dim) const;
        /**
         * Get the number of constrained derivatives in
         * a certain dimension
         * @param dim
         * @return
         */
        int getNumConstraints(int dim) const;

        /**
         * Build the required matrices for the QP problem using the
         * constraints currently added to the generator.
         */
        void buildProblem();

        /**
         * Send off the problem to a solver and run the optimization
         * algorithm
         * @param dim
         * @return If the optimization worked
         */
        bool solveProblem(int dim, Solver solver);

    private:
        static const int k_r_ = 4;   // Order of the derivative of the position
        static const int n_ = 6;     // Order of the polynomials describing the trajectory
        static const int n_coeffs_ = n_ + 1; // Number of coefficients in a polynomial
        static const int states_ = 3;// Number of states in the problem
        std::vector<TrajectoryConstraint> keyframes_;   // Constraints on the trajectory
                                                        // including initial conditions
        MatrixXd H_[states_];               // Cost matrices
        MatrixXd A_fixed_[states_];         // Fixed constraints matrix
        VectorXd b_fixed_[states_];         // b vector in Ax = b
        MatrixXd A_continuity_[states_];    // Continuity constraints matrix
        VectorXd b_continuity_[states_];    // b vector in Ax = b
        VectorXd solution_[states_];        // Solution vector

        const int X = 0;
        const int Y = 1;
        const int Z = 2;
        bool problemBuilt_;

        bool solveProblemOoqp(int dim);
        bool solveProblemGurobi(int dim);
        bool solveProblemQld(int dim);

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
