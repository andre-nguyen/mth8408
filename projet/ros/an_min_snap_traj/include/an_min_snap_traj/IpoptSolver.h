#ifndef IPOPT_SOLVER_H
#define IPOPT_SOLVER_H
#include "IpoptProblem.h"

namespace an_min_snap_traj
{
  /*************************************************************
  IpoptSolver : ipopt solver
  This class solves general programming.
  *************************************************************/

  class IpoptSolver
  {
  private:
    Ipopt::SmartPtr<Ipopt::TNLP> m_problem;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> m_solver;
    bool isWarm = false;
  public:
    IpoptSolver(GeneralProgramming* gp);
    ~IpoptSolver();

    void Solve();
  };
};

#endif
