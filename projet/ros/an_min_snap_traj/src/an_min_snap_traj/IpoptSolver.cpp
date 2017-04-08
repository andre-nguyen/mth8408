#include "an_min_snap_traj/IpoptSolver.h"

using namespace Ipopt;
namespace an_min_snap_traj {
/*************************************************************
        Constructor
*************************************************************/
IpoptSolver::
IpoptSolver(GeneralProgramming *gp)
  : m_problem(new IpoptProblem(gp)), m_solver(new IpoptApplication())
{
  m_solver->Options()->SetStringValue("mu_strategy", "adaptive");
  m_solver->Options()->SetStringValue("jac_d_constant", "yes");
  m_solver->Options()->SetStringValue("jac_c_constant", "yes");
  m_solver->Options()->SetStringValue("hessian_constant", "yes");
  m_solver->Options()->SetStringValue("mehrotra_algorithm", "yes");
  m_solver->Options()->SetIntegerValue("print_level", 5);
  m_solver->Options()->SetIntegerValue("max_iter", 200);
  m_solver->Options()->SetNumericValue("tol", 1e-8);
}

/*************************************************************
        Destructor
*************************************************************/
IpoptSolver::
~IpoptSolver()
{}

/*************************************************************
        solve function
*************************************************************/
void
IpoptSolver::
Solve()
{
  if (!isWarm)
  {
    isWarm = true;
    m_solver->Initialize();
    m_solver->OptimizeTNLP(m_problem);
  }
  else m_solver->ReOptimizeTNLP(m_problem);
}
}
