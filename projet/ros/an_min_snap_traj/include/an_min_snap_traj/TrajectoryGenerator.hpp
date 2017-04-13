#ifndef GEN_TRAJ_HPP
#define GEN_TRAJ_HPP

#include <vector>
#include <Eigen/Core>
#include "an_min_snap_traj/TrajectoryConstraint.hpp"
#include "an_min_snap_traj/TrajectorySegment.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    class TrajectoryGenerator {
    public:
        enum Solver {OOQP, GUROBI, QLD, QUADPROG, IPOPT, QPOASES};

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

        /**
         * Gets the full constraints matrix Aeq including fixed constraints
         * and continuity constraints
         * @param dim
         * @return
         */
        MatrixXd getConstraintMatrix(int dim) const;

        /**
         * Gets the beq vector in Aeq * x = beq. Including fixed
         * constraints and continuity constraints.
         * @param dim
         * @return
         */
        VectorXd getConstraintVector(int dim) const;

        /**
         * Get the fixed constraints matrix of Aeq x = b
         * @param dim
         * @return
         */
        MatrixXd getFixedConstraintMatrix(int dim) const;

        /**
         * Get the fixed constraints b of Aeq x = b
         * @param dim
         * @return
         */
        VectorXd getFixedConstraintVector(int dim) const;

        /**
         * Get the continuity constraints matrix of Aeq x = b
         * @param dim
         * @return
         */
        MatrixXd getContinuityConstraintMatrix(int dim) const;

        /**
         * Get the continuity constraints vector b of Aeq x = b
         * @param dim
         * @return
         */
        VectorXd getContinuityConstraintVector(int dim) const;

        /**
         * Get the raw solution straight from the optimizer
         * Will be a vector containing all the coefficients of the
         * polynomials
         * @param dim
         * @return
         */
        VectorXd getSolution(int dim) const;

        std::vector<Vector3d> discretizeSolution();

        std::vector<Vector3d> getDiscreteSolution(Derivative der);

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
         * Send off the problem in all states to a solver and run optimization
         * @param solver
         * @return
         */
        bool solveProblem(Solver solver);

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
        const double dt_ = 0.1;
        std::vector<TrajectoryConstraint> keyframes_;       // Constraints on the trajectory
                                                            // including initial conditions
        MatrixXd H_[states_];                               // Cost matrices
        MatrixXd A_fixed_[states_];                         // Fixed constraints matrix
        VectorXd b_fixed_[states_];                         // b vector in Ax = b
        MatrixXd A_continuity_[states_];                    // Continuity constraints matrix
        VectorXd b_continuity_[states_];                    // b vector in Ax = b
        VectorXd solution_[states_];                        // Solution vector
        std::vector<TrajectorySegment> solutionSegments_;   // Polynomial segments

        const int X = 0;
        const int Y = 1;
        const int Z = 2;
        bool problemBuilt_;
        bool isDiscretized_;

        // TODO pick only 1 solver after the class is done
        bool solveProblemOoqp(int dim);
        bool solveProblemGurobi(int dim);
        bool solveProblemQld(int dim);
        bool solveProblemQuadprog(int dim);
        bool solveProblemIPOPT(int dim);
        bool solveProblemqpOASES(int dim);

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
