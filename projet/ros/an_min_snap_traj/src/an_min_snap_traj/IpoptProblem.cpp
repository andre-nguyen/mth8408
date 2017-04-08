#include "an_min_snap_traj/IpoptProblem.h"

using namespace Ipopt;
namespace an_min_snap_traj {
/*************************************************************
        ArrayToEigenVector function
*************************************************************/
Eigen::VectorXd ArrayToEigenVector(const double *x, int n)
{
  Eigen::VectorXd ret(n);

  for (int i = 0; i < n; i++) ret[i] = x[i];

  return ret;
}

/*************************************************************
        Constructor
*************************************************************/
IpoptProblem::
IpoptProblem(GeneralProgramming *_gp)
  : problem(_gp)
{}

/*************************************************************
        Destructor
 ******************************************r*******************/
IpoptProblem::
~IpoptProblem()
{
  delete problem;
}

/*************************************************************
        get_nlp_info function
*************************************************************/
bool
IpoptProblem::
get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
             Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  n = problem->GetNumDimension();
  m = problem->GetNumEqualityConstraint();

  nnz_jac_g =
    problem->EvalEqualityConstraintJacobian(Eigen::VectorXd::Constant(n,
                                                                      0.0)).
    nonZeros();
  nnz_h_lag =
    problem->EvalObjectiveHessian(Eigen::VectorXd::Constant(n, 0.0)).nonZeros();

  index_style = TNLP::C_STYLE;
  return true;
}

/*************************************************************
        get_bounds_info function
*************************************************************/
bool
IpoptProblem::
get_bounds_info(Index n, Number *x_l, Number *x_u,
                Index m, Number *g_l, Number *g_u)
{
  const Eigen::VectorXd& lb = problem->GetLowerBoundary();
  const Eigen::VectorXd& ub = problem->GetUpperBoundary();
  const Eigen::VectorXd& b = problem->GetEqualityconstraints();

  for (int i = 0; i < n; i++) x_l[i] = lb[i];

  for (int i = 0; i < n; i++) x_u[i] = ub[i];

  for (int i = 0; i < m; i++) g_l[i] = b[i];

  for (int i = 0; i < m; i++) g_u[i] = b[i];

  return true;
}

/*************************************************************
        get_starting_point function
*************************************************************/
bool
IpoptProblem::
get_starting_point(Index n, bool init_x, Number *x,
                   bool init_z, Number *z_L, Number *z_U,
                   Index m, bool init_lambda,
                   Number *lambda)
{
  const Eigen::VectorXd& ig = problem->GetInitialGuess();

  for (int i = 0; i < n; i++) x[i] = ig[i];


  return true;
}

/*************************************************************
        eval_f function
*************************************************************/
bool
IpoptProblem::
eval_f(Index n, const Number *x, bool new_x, Number& obj_value)
{
  obj_value = problem->EvalObjective(ArrayToEigenVector(x, n));

  return true;
}

/*************************************************************
        eval_grad_f function
*************************************************************/
bool
IpoptProblem::
eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f)
{
  Eigen::VectorXd grad = problem->EvalObjectiveGrad(ArrayToEigenVector(x, n));

  for (int i = 0; i < n; i++) grad_f[i] = grad(i);

  return true;
}

/*************************************************************
        eval_g function
*************************************************************/
bool
IpoptProblem::
eval_g(Index n, const Number *x, bool new_x, Index m, Number *g)
{
  Eigen::VectorXd eqg = problem->EvalEqualityConstraint(ArrayToEigenVector(x, n));

  for (int i = 0; i < m; i++) g[i] = eqg(i);

  return true;
}

/*************************************************************
        eval_jac_g function
*************************************************************/
bool
IpoptProblem::
eval_jac_g(Ipopt::Index         n,
           const Ipopt::Number *x,
           bool                 new_x,
           Ipopt::Index         m,
           Ipopt::Index         nele_jac,
           Ipopt::Index        *iRow,
           Ipopt::Index        *jCol,
           Ipopt::Number       *values)
{
  int nnz = 0;

  if (values == NULL)
  {
    Eigen::SparseMatrix<double> J = problem->EvalEqualityConstraintJacobian(
      Eigen::VectorXd::Constant(n, 0.0));

    for (int k = 0; k < J.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(J, k); it; ++it)
      {
        iRow[nnz]   = it.row();
        jCol[nnz++] = it.col();
      }
  }
  else
  {
    Eigen::SparseMatrix<double> J = problem->EvalEqualityConstraintJacobian(
      ArrayToEigenVector(x, n));

    for (int k = 0; k < J.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(J, k); it;
           ++it) values[nnz++] = it.value();
  }

  return true;
}

/*************************************************************
        eval_h function
*************************************************************/
bool
IpoptProblem::
eval_h(Index n, const Number *x, bool new_x,
       Number obj_factor, Index m, const Number *lambda,
       bool new_lambda, Index nele_hess, Index *iRow,
       Index *jCol, Number *values)
{
  int nnz = 0;

  if (values == NULL)
  {
    Eigen::SparseMatrix<double> H = problem->EvalObjectiveHessian(
      Eigen::VectorXd::Constant(n, 0.0));

    for (int k = 0; k < H.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(H, k); it; ++it)
      {
        iRow[nnz]   = it.row();
        jCol[nnz++] = it.col();
      }
  }
  else
  {
    Eigen::SparseMatrix<double> H = problem->EvalObjectiveHessian(
      ArrayToEigenVector(x, n));

    for (int k = 0; k < H.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(H, k); it; ++it) {
        values[nnz++] = it.value() * obj_factor;
      }
  }

  return true;
}

/*************************************************************
        finalize_solution function
*************************************************************/
void
IpoptProblem::
finalize_solution(SolverReturn status,
                  Index n, const Number *x, const Number *z_L, const Number *z_U,
                  Index m, const Number *g, const Number *lambda,
                  Number obj_value, const IpoptData *ip_data,
                  IpoptCalculatedQuantities *ip_cq)
{
  problem->SetOptimalSolution(ArrayToEigenVector(x, n));
}
}
