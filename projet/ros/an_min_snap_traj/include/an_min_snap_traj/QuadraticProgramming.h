#ifndef __QUADRATIC_PROGRAMMING_H__
#define __QUADRATIC_PROGRAMMING_H__
#include "GeneralProgramming.h"

namespace an_min_snap_traj
{
/*************************************************************
   QuadraticPrgramming : Quadratic Programming container

   This class contains the quadratic problem. This class
   doesn't contains a solver, only containing the problem.

   The problem that this container have is followed.

   min      0.5*x^T*H*x + x^t*c
   x

   sub.to.  A*x + b = 0

*************************************************************/
class QuadraticProgramming : public GeneralProgramming
{
public:
        QuadraticProgramming(const unsigned int n, const unsigned int m);
        ~QuadraticProgramming();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
        void SetH(const std::vector<Eigen::Triplet<double> >& h_trip);
        void Setc(const Eigen::VectorXd& c);
        void SetA(const std::vector<Eigen::Triplet<double> >& a_trip);
        void SetA(const Eigen::MatrixXd& A);
        void Setb(const Eigen::VectorXd& b);

public:
        const Eigen::SparseMatrix<double>&  GetH();
        const Eigen::SparseMatrix<double>&  GetA();
        const Eigen::VectorXd&              Getc();
        const Eigen::VectorXd&              Getb();
        virtual const Eigen::VectorXd& GetEqualityconstraints() const;
public:
        virtual double EvalObjective(const Eigen::VectorXd& x) const;
        virtual Eigen::VectorXd EvalObjectiveGrad(const Eigen::VectorXd& x) const;
        virtual Eigen::SparseMatrix<double> EvalObjectiveHessian(const Eigen::VectorXd& x) const;

        virtual Eigen::VectorXd               EvalEqualityConstraint(const Eigen::VectorXd& x) const;
        virtual Eigen::SparseMatrix<double>   EvalEqualityConstraintJacobian(const Eigen::VectorXd& x) const;

protected:
        Eigen::SparseMatrix<double> H;
        Eigen::SparseMatrix<double> A;
        Eigen::VectorXd c;
        Eigen::VectorXd b;


};
};
#endif
