#include "an_min_snap_traj/GeneralProgramming.h"

namespace an_min_snap_traj {
/*************************************************************
   Constructor
*************************************************************/
GeneralProgramming::
GeneralProgramming(const unsigned int _n,
                   const unsigned int _m)
  : n(_n), m(_m),
    lb(Eigen::VectorXd(_n)), ub(Eigen::VectorXd(_n)),
    optimal_solution(Eigen::VectorXd::Constant(_n, 0.0)),
    initial_guess(Eigen::VectorXd::Constant(_n, 0.0))
{}

/*************************************************************
   Destructor
*************************************************************/
GeneralProgramming::
~GeneralProgramming()
{}

/*************************************************************
   GetNumDimension function
*************************************************************/
unsigned int
GeneralProgramming::
GetNumDimension() const
{
  return n;
}

/*************************************************************
   GetNumEqualityConstraint function
*************************************************************/
unsigned int
GeneralProgramming::
GetNumEqualityConstraint() const
{
  return m;
}

/*************************************************************
   GetOptimalSolution function
*************************************************************/
const Eigen::VectorXd&
GeneralProgramming::
GetOptimalSolution() const
{
  return optimal_solution;
}

/*************************************************************
   GetLowerBoundary function
*************************************************************/
const Eigen::VectorXd&
GeneralProgramming::
GetLowerBoundary() const
{
  return lb;
}

/*************************************************************
   GetUpperBoundary function
*************************************************************/
const Eigen::VectorXd&
GeneralProgramming::
GetUpperBoundary() const
{
  return ub;
}

/*************************************************************
   GetInitialGuess function
*************************************************************/
const Eigen::VectorXd&
GeneralProgramming::
GetInitialGuess() const
{
  return initial_guess;
}

/*************************************************************
   SetOptimalSolution function
*************************************************************/
void
GeneralProgramming::
SetOptimalSolution(const Eigen::VectorXd& x)
{
  optimal_solution = x;
}

/*************************************************************
   SetInitialGuess function
*************************************************************/
void
GeneralProgramming::
SetInitialGuess(const Eigen::VectorXd& x)
{
  initial_guess = x;
}

/*************************************************************
   SetLowerBoundary function
*************************************************************/
void
GeneralProgramming::
SetLowerBoundary(const Eigen::VectorXd& lb)
{
  this->lb = lb;
}

/*************************************************************
   SetUpperBoundary function
*************************************************************/
void
GeneralProgramming::
SetUpperBoundary(const Eigen::VectorXd& ub)
{
  this->ub = ub;
}
}
