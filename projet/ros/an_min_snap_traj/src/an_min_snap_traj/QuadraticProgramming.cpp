#include "an_min_snap_traj/QuadraticProgramming.h"
namespace an_min_snap_traj {
/*************************************************************
        Constructor
*************************************************************/
QuadraticProgramming::
QuadraticProgramming(const unsigned int _n,
                     const unsigned int _m)
  : GeneralProgramming(_n, _m),
    H(Eigen::SparseMatrix<double>(_n, _n)), c(Eigen::VectorXd(_n)),
    A(Eigen::SparseMatrix<double>(_m, _n)), b(Eigen::VectorXd(_m))
{}

/*************************************************************
        Destructor
*************************************************************/
QuadraticProgramming::
~QuadraticProgramming()
{}

/*************************************************************
        SetH function
*************************************************************/
void
QuadraticProgramming::
SetH(const std::vector<Eigen::Triplet<double> >& h_trip)
{
  H.setFromTriplets(h_trip.cbegin(), h_trip.cend());
}

/*************************************************************
        Setc function
*************************************************************/
void
QuadraticProgramming::
Setc(const Eigen::VectorXd& c)
{
  this->c = c;
}

/*************************************************************
        SetA function

   Use this function when the A is sparse.
*************************************************************/
void
QuadraticProgramming::
SetA(const std::vector<Eigen::Triplet<double> >& a_trip)
{
  A.setFromTriplets(a_trip.cbegin(), a_trip.cend());
}

/*************************************************************
        SetA function

   Use this function when the A is dense.
*************************************************************/
void
QuadraticProgramming::
SetA(const Eigen::MatrixXd& _A)
{
  std::vector<Eigen::Triplet<double> > a_trip;
  a_trip.reserve(_A.rows() * _A.cols());

  for (int i = 0; i < _A.rows(); i++)
    for (int j = 0; j < _A.cols();
         j++) a_trip.push_back(Eigen::Triplet<double>(i, j, _A(i, j)));


  this->SetA(a_trip);
}

/*************************************************************
        Setb function
*************************************************************/
void
QuadraticProgramming::
Setb(const Eigen::VectorXd& b)
{
  this->b = b;
}

const Eigen::SparseMatrix<double>&
QuadraticProgramming::
GetH()
{
  return H;
}

const Eigen::SparseMatrix<double>&
QuadraticProgramming::
GetA()
{
  return A;
}

const Eigen::VectorXd&
QuadraticProgramming::
Getc()
{
  return c;
}

const Eigen::VectorXd&
QuadraticProgramming::
Getb()
{
  return b;
}

const Eigen::VectorXd&
QuadraticProgramming::
GetEqualityconstraints() const
{
    return b;
}

/*************************************************************
        EvalObjective function
*************************************************************/
double
QuadraticProgramming::
EvalObjective(const Eigen::VectorXd& x) const
{
  return 0.5 * x.dot(H * x) + x.dot(c);
}

/*************************************************************
        EvalObjectiveGrad function
*************************************************************/
Eigen::VectorXd
QuadraticProgramming::
EvalObjectiveGrad(const Eigen::VectorXd& x) const
{
  return H * x + c;
}

/*************************************************************
        EvalObjectiveHessian function
*************************************************************/
Eigen::SparseMatrix<double>
QuadraticProgramming::
EvalObjectiveHessian(const Eigen::VectorXd& x) const
{
  return H;
}

/*************************************************************
        EvalEqualityConstraint function
*************************************************************/
Eigen::VectorXd
QuadraticProgramming::
EvalEqualityConstraint(const Eigen::VectorXd& x) const
{
  return A * x + b;
}

/*************************************************************
        EvalEqualityConstraintJacobian function
*************************************************************/
Eigen::SparseMatrix<double>
QuadraticProgramming::
EvalEqualityConstraintJacobian(const Eigen::VectorXd& x) const
{
  return A;
}
}
