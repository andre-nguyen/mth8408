/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2017 Mobile Robotics and Autonomous Systems Laboratory (MRASL),
 * Polytechnique Montreal. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

/**
 * @author  Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 * @file    IpoptAdapter.hpp
 * @details Adapts IPOPT to solve a QP problem
 */

#ifndef AN_MIN_SNAP_TRAJ_IPOPTADAPTER_HPP
#define AN_MIN_SNAP_TRAJ_IPOPTADAPTER_HPP

#include "IpTNLP.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "an_min_snap_traj/OptimizationStats.hpp"

using namespace Ipopt;
using namespace Eigen;

namespace an_min_snap_traj {
/**
 * The IpoptAdapter provides an easy way of solving a QP problem using Ipopt.
 * Ipopt usually solves an nlp of the form
 *        min     f(x)
 *      x in R^n
 *
 *      s.t.       g_L <= g(x) <= g_U
 *                 x_L <=  x   <= x_U
 *
 * We adapt this to a QP form such that
 *      min     0.5 x' H x + c' x
 *    x in R^n
 *      s.t.    Ax = b
 *              x_L <= x <= x_U
 *
 * The gradient of this form is
 *      nabla f(x) = H x + c
 * The jacobian of the constraints is
 *      Jac(g(x)) = A
 * The hessian of the objective function is
 *      Hess(f(x)) = H
 */
class IpoptAdapter : public TNLP {
public:
    static constexpr double PLUS_INFINITY  = 1.0e19;
    static constexpr double MINUS_INFINITY = -1.0e19;
    IpoptAdapter(MatrixXd H, VectorXd c, MatrixXd A, VectorXd b,
                 VectorXd lb,
                 VectorXd ub);

    virtual ~IpoptAdapter();

    VectorXd getSolution() const {return x_;}

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the nlp */
    virtual bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                              Ipopt::Index &nnz_h_lag,
                              IndexStyleEnum &Index_style);

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(Ipopt::Index n, Number *x_l, Number *x_u,
                                 Ipopt::Index m, Number *g_l, Number *g_u);

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Number *x,
                                    bool init_z, Number *z_L, Number *z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    Number *lambda);

    /** Method to return the objective value */
    virtual bool
    eval_f(Ipopt::Index n, const Number *x, bool new_x, Number &obj_value);

    /** Method to return the gradient of the objective */
    virtual bool
    eval_grad_f(Ipopt::Index n, const Number *x, bool new_x, Number *grad_f);

    /** Method to return the constraint residuals */
    virtual bool
    eval_g(Ipopt::Index n, const Number *x, bool new_x, Ipopt::Index m, Number *g);

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(Ipopt::Index n, const Number *x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                            Ipopt::Index *jCol,
                            Number *values);

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    virtual bool eval_h(Ipopt::Index n, const Number *x, bool new_x,
                        Number obj_factor, Ipopt::Index m, const Number *lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                        Ipopt::Index *jCol, Number *values);

    virtual void finalize_solution(SolverReturn status,
                                   Ipopt::Index n, const Number* x, const Number* z_L, const Number* z_U,
                                   Ipopt::Index m, const Number* g, const Number* lambda,
                                   Number obj_value,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq);

private:
    // Block default compiler methods
    IpoptAdapter(const IpoptAdapter &);

    IpoptAdapter &operator=(const IpoptAdapter &);

    MatrixXd H_, A_;
    SparseMatrix<double> H_sparse_, A_sparse_;
    VectorXd c_, b_, lb_, ub_, x_;
    long n_vars_;    // number of variables
    long n_constr_;  // number of constraints

    Ipopt::SolverReturn term_condition_;
    OptimizationStats stats_;
};

} // namespace an_min_snap_traj

#endif //AN_MIN_SNAP_TRAJ_IPOPTADAPTER_HPP
