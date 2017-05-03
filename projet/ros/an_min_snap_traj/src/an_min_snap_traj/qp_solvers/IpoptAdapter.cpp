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
 * @file    IpoptAdapter.cpp
 * @details Adapts IPOPT to solve a QP problem
 */
#include <IpIpoptData.hpp>
#include <Eigen/Dense>

#include "an_min_snap_traj/qp_solvers/IpoptAdapter.hpp"

namespace an_min_snap_traj {

    IpoptAdapter::IpoptAdapter(MatrixXd H, VectorXd c, MatrixXd A, VectorXd b,
                               VectorXd lb, VectorXd ub) : H_(H), c_(c), A_(A),
    b_(b), lb_(lb), ub_(ub){
        n_vars_ = H_.cols();
        n_constr_ = b.rows();

        // sanity check
        assert(c.rows() == n_vars_);
        assert(A.cols() == n_vars_);
        assert(A.rows() == n_constr_);
        assert(lb.rows() == n_vars_);
        assert(ub.rows() == n_vars_);

        H_sparse_ = H_.sparseView();
        A_sparse_ = A_.sparseView();
    }

    IpoptAdapter::~IpoptAdapter() {}

    /***************************************************************************
     * Overloaded TNLP implementation
     **************************************************************************/
    bool IpoptAdapter::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &Index_style) {
        n = n_vars_;
        m = n_constr_;

        nnz_jac_g = A_sparse_.nonZeros();
        nnz_h_lag = H_sparse_.nonZeros();
        Index_style = TNLP::C_STYLE;

        return true;
    }

    bool IpoptAdapter::get_bounds_info(Ipopt::Index n, Number *x_l, Number *x_u,
                                 Ipopt::Index m, Number *g_l, Number *g_u){
        assert(n == n_vars_);
        assert(m == n_constr_);
        for(int i = 0; i < n; ++i) {
            x_l[i] = lb_(i);
            x_u[i] = ub_(i);
        }

        for(int i = 0; i < m; ++i) {
            g_l[i] = g_u[i] = 0;
        }

        return true;
    }

    /** Method to return the starting point for the algorithm */
    bool IpoptAdapter::get_starting_point(Ipopt::Index n, bool init_x, Number *x,
                                    bool init_z, Number *z_L, Number *z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    Number *lambda){
        assert(init_x == true);
        assert(init_z == false);
        assert(init_lambda == false);

        for(int i = 0; i < n; ++i){
            x[i] = 0.0;
        }

        return true;
    }

    /** Method to return the objective value */
    bool IpoptAdapter::eval_f(Ipopt::Index n, const Number *x, bool new_x,
                              Number &obj_value){
        assert(n = n_vars_);

        // convert x to eigen
        VectorXd eig_x(n);
        for(int i = 0; i < n; ++i) eig_x(i) = x[i];
        obj_value = (0.5 * (eig_x.transpose() * H_ * eig_x))(0); // quadratic
        // term
        obj_value = obj_value + (c_.transpose() * eig_x);   // linear term*/
        return true;
    }

    /** Method to return the gradient of the objective */
    bool IpoptAdapter::eval_grad_f(Ipopt::Index n, const Number *x, bool new_x,
                                   Number *grad_f){
        assert(n = n_vars_);
        // convert x to eigen
        VectorXd eig_x(n);
        for(int i = 0; i < n; ++i) eig_x(i) = x[i];

        VectorXd grad = H_ * eig_x + c_;
        for(int i = 0; i < n; ++i) grad_f[i] = grad(i);

        return true;
    }

    /** Method to return the constraint residuals */
    bool IpoptAdapter::eval_g(Ipopt::Index n, const Number *x, bool new_x,
                              Ipopt::Index m, Number *g){
        assert(n = n_vars_);
        assert(m = n_constr_);
        VectorXd eig_x(n);
        for(int i = 0; i < n; ++i) eig_x(i) = x[i];

        VectorXd constraint = A_ * eig_x - b_;
        for(int i = 0; i < m; ++i) g[i] = constraint(i);

        return true;
    }

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    bool IpoptAdapter::eval_jac_g(Ipopt::Index n, const Number *x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                            Ipopt::Index *jCol,
                            Number *values){
        assert(n == n_vars_);
        assert(m == n_constr_);
        if((values == NULL) && (iRow != NULL) && (jCol != NULL)) {
            // return structure
            std::cout << "A sparse \n" << A_sparse_;
            std::cout << "iteration form: \n";
            int i = 0;
            for(int k = 0; k < A_sparse_.outerSize(); ++k) {
                for(SparseMatrix<double>::InnerIterator it(A_sparse_, k); it;
                    ++it) {
                    iRow[i] = it.row();
                    jCol[i] = it.col();
                    printf("(%ld, %ld) ", it.row(), it.col());
                    i += 1;
                }
            }
            printf("\n");
        } else {
            // return jacobian
            int i = 0;
            for(int k = 0; k < A_sparse_.outerSize(); ++k) {
                for(SparseMatrix<double>::InnerIterator it(A_sparse_, k); it;
                    ++it) {
                    values[i] = it.value();
                    i += 1;
                }
            }
        }

        return true;
    }

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    bool IpoptAdapter::eval_h(Ipopt::Index n, const Number *x, bool new_x,
                        Number obj_factor, Ipopt::Index m, const Number *lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                        Ipopt::Index *jCol, Number *values) {
        assert(n == n_vars_);
        assert(m == n_constr_);
        if((values == NULL) && (iRow != NULL) && (jCol != NULL)) {
            int i = 0;
            std::cout << "H sparse \n" << H_sparse_;
            std::cout << "H iteration form: \n";
            for(int k = 0; k < H_sparse_.outerSize(); ++k) {
                for(SparseMatrix<double>::InnerIterator it(H_sparse_, k); it;
                    ++it) {
                    iRow[i] = it.row();
                    jCol[i] = it.col();
                    printf("(%ld, %ld) ", it.row(), it.col());
                    i += 1;
                }
            }
            printf("\n");
        } else {
            int i = 0;
            for(int k = 0; k < H_sparse_.outerSize(); ++k) {
                for(SparseMatrix<double>::InnerIterator it(H_sparse_, k); it;
                    ++it) {
                    values[i] = it.value();
                    i += 1;
                }
            }
        }

        return true;
    }

    void IpoptAdapter::finalize_solution(SolverReturn status, Ipopt::Index n,
                                         const Number *x, const Number *z_L,
                                         const Number *z_U, Ipopt::Index m,
                                         const Number *g, const Number *lambda,
                                         Number obj_value,
                                         const IpoptData *ip_data,
                                         IpoptCalculatedQuantities *ip_cq) {
        term_condition_ = status;
        stats_.success_ = (term_condition_ == SUCCESS);
        stats_.iters_ = (int)ip_data->iter_count();
        for(int i = 0; i < n; ++i) x_(i) = x[i];
    }
}