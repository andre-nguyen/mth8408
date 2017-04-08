#ifndef GENERAL_PROGRAMMING_H
#define GENERAL_PROGRAMMING_H
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <vector>

namespace an_min_snap_traj {
/*************************************************************
   GeneralPrgramming : general non-linear problem container
   This class contains the general non-linear problem. This class
   doesn't contains a solver, only containing the problem.
   Since this class is pure virtual class, you have to inherite this
   class to use it. (See Quadratic Programming class)
*************************************************************/
class GeneralProgramming
{
public:
        GeneralProgramming(const unsigned int n,const unsigned int m);
        ~GeneralProgramming();
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
        unsigned int GetNumDimension() const;
        unsigned int GetNumEqualityConstraint() const;

        const Eigen::VectorXd& GetOptimalSolution() const;
        const Eigen::VectorXd& GetLowerBoundary() const;
        const Eigen::VectorXd& GetUpperBoundary() const;
        const Eigen::VectorXd& GetInitialGuess() const;
        virtual const Eigen::VectorXd& GetEqualityconstraints() const = 0;

        void SetOptimalSolution(const Eigen::VectorXd& x);
        void SetInitialGuess(const Eigen::VectorXd& x);
        void SetLowerBoundary(const Eigen::VectorXd& _lb);
        void SetUpperBoundary(const Eigen::VectorXd& _ub);

        virtual double EvalObjective(const Eigen::VectorXd& x) const = 0;
        virtual Eigen::VectorXd EvalObjectiveGrad(const Eigen::VectorXd& x) const = 0;
        virtual Eigen::SparseMatrix<double> EvalObjectiveHessian(const Eigen::VectorXd& x) const = 0;

        virtual Eigen::VectorXd EvalEqualityConstraint(const Eigen::VectorXd& x) const = 0;
        virtual Eigen::SparseMatrix<double> EvalEqualityConstraintJacobian(const Eigen::VectorXd& x) const = 0;
protected:
        /*************************************************************
           n : dim of variables
           m : dim of equality constraints
           lb, ub : lower bound and upper bound of variables
        *************************************************************/
        unsigned int n,m;
        Eigen::VectorXd lb,ub;
        Eigen::VectorXd optimal_solution;
        Eigen::VectorXd initial_guess;
};
};
#endif
