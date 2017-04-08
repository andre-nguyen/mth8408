//
// Created by andre on 4/6/17.
//

#include <Eigen/Sparse>
#include <iostream>
#include "an_min_snap_traj/IpoptAdapter.h"

using namespace Ipopt;
// Dont use Eigen namespace, conflicts with Index type

namespace an_min_snap_traj {
    IpoptAdapter::IpoptAdapter(Eigen::MatrixXd H, Eigen::MatrixXd Aeq, Eigen::VectorXd beq) :
            H_(H), Aeq_(Aeq), beq_(beq) {
        // nnz_jac_g, anything that is zero right now will always be zero, count the non zero terms
        // of Aeq
        Eigen::SparseMatrix<double> aeq_sparse = Aeq_.sparseView();
        nnz_jac_g_ = aeq_sparse.nonZeros();

        // nnz_h_lag, just take non zeros of lower triangular H
        Eigen::MatrixXd h_trig = H_.triangularView<Eigen::Lower>();
        Eigen::SparseMatrix<double> h_trig_sparse = h_trig.sparseView();
        nnz_h_lag_ = h_trig_sparse.nonZeros();
        // keep in memory what positions were non-zero
        for(int k = 0; k < h_trig_sparse.outerSize(); ++k){
            for(Eigen::SparseMatrix<double>::InnerIterator it(h_trig_sparse, k);
                    it; ++it) {
                Eigen::Triplet<double> t(it.row(), it.col(), it.value());
                lagrangian_triplets.push_back(t);
            }
        }
    }

    IpoptAdapter::~IpoptAdapter() {}

    bool IpoptAdapter::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag,
                                    IndexStyleEnum &index_style) {
        n = H_.rows();
        m = beq_.rows();
        nnz_jac_g = nnz_jac_g_;
        nnz_h_lag = nnz_h_lag_;

        index_style = TNLP::C_STYLE;
        return true;
    }

    bool IpoptAdapter::get_bounds_info(Index n, Number *x_l, Number *x_u, Index m, Number *g_l, Number *g_u) {
        assert(n == H_.rows());
        assert(m == beq_.rows());

        for(Index i = 0; i < n; ++i) {
            x_l[i] = std::numeric_limits<double>::min();
            x_u[i] = std::numeric_limits<double>::max();
        }

        // since we only have equality constraints, gl and gu are the same
        for(int i = 0; i < m; ++i){
            g_l[i] = beq_(i);
            g_u[i] = beq_(i);
        }

        return true;
    }

    bool IpoptAdapter::get_starting_point(Index n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U,
                                          Index m, bool init_lambda, Number *lambda) {
        assert(init_x == true);
        assert(init_z == false);
        assert(init_lambda == false);
        for(int i = 0; i < n; ++i) {
            x[i] = 1.0;
        }

        // XXX incomplete


    }

    bool IpoptAdapter::eval_f(Index n, const Number *x, bool new_x, Number &obj_value) {
        assert(n == H_.rows());
        // Copy x
        Eigen::VectorXd c(n);
        for(Index i = 0; i < n; ++i) {
            c(i) = x[i];
        }

        // Compute 0.5 cHc'
        auto result = 0.5 * c.transpose() * H_ * c;
        obj_value = result;
        return true;
    }

    bool IpoptAdapter::eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) {
        assert(n == H_.rows());
        // Copy x
        Eigen::VectorXd c(n);
        for(Index i = 0; i < n; ++i) {
            c(i) = x[i];
        }

        // Gradient of f = 0.5 cHc'
        // 0.5 H'c + Hc
        Eigen::VectorXd result = 0.5 * ((H_.transpose() * c) + (H_ * c));
        for(int i = 0; i < n; ++i) {
            grad_f[i] = result(i);
        }
        return true;
    }

    bool IpoptAdapter::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
        assert(n == H_.rows());
        assert(m == beq_.rows());
        // Copy x
        Eigen::VectorXd c(n);
        for(Index i = 0; i < n; ++i) {
            c(i) = x[i];
        }

        // Aeq * x = g(x)
        Eigen::VectorXd res = Aeq_ * c;
        for(int i = 0; i < m; ++i) {
            g[i] = res(i);
        }
        return true;
    }

    bool IpoptAdapter::eval_jac_g(Index n, const Number *x, bool new_x, Index m, Index nele_jac, Index *iRow,
                                  Index *jCol, Number *values) {
        // Construct Jacobian
        // Basically g(x) = Aeq * x
        // and the jacobian J = Aeq, easy peasy, we already have it
        Eigen::SparseMatrix<double> aeq_sparse;
        aeq_sparse = Aeq_.sparseView();

        if(values == NULL) {
            // get structure using inner iterator
            int i = 0;
            for (int k=0; k < aeq_sparse.outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(aeq_sparse,k); it; ++it)
                {
                    iRow[i] = it.row(); jCol[i] = it.col();
                    i++;
                }
            }
        } else {
            // get values
            int i = 0;
            for (int k=0; k < aeq_sparse.outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(aeq_sparse,k); it; ++it)
                {
                    values[i] = it.value();
                    i++;
                }
            }
        }

        return true;
    }

    bool IpoptAdapter::eval_h(Index n, const Number *x, bool new_x, Number obj_factor, Index m, const Number *lambda,
                              bool new_lambda, Index nele_hess, Index *iRow, Index *jCol, Number *values) {
        /**
         *  For a qp problem 0.5 x'Qx + c'x
         *      Subject to Ax - b = 0
         *  The lagrangian is 0.5 x'Qx + c'x - lambda'(Ax-b)
         *  The hessian of the lagrangian is Qx + c - A'lambda
         *  IPOPT also has a multiplier sigma (aka obj_factor) so the true hessian becomes
         *      sigma(Qx + c) - A'lambda
         *  Of course, our problem has c = 0.
         */
        assert(n = H_.rows());
        assert(m == beq_.rows());
        assert(nele_hess == lagrangian_triplets.size());

        Eigen::MatrixXd h_trig = H_.triangularView<Eigen::Lower>();
        Eigen::SparseMatrix<double> h_trig_sparse = h_trig.sparseView();
        if(values == NULL) {
            int i = 0;
            for (int k=0; k < h_trig_sparse.outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(h_trig_sparse,k); it; ++it)
                {
                    iRow[i] = it.row(); jCol[i] = it.col();
                    i++;
                }
            }
        } else {
            // Build x vector
            Eigen::VectorXd e_x(n);
            for(int i = 0; i < n; ++i) {
                e_x(i) = x[i];
            }

            // Build lambda vector
            Eigen::VectorXd e_lambda(n);
            for(int i = 0; i < m; ++i) {
                e_lambda(i) = lambda[i];
            }

            // sigma(Qx + c) - A'lambda
            Eigen::MatrixXd h_lag = (obj_factor * (H_ * e_x)) - (Aeq_.transpose() * e_lambda);
            Eigen::MatrixXd h_lag_trig = h_lag.triangularView<Eigen::Lower>();
            // Might as well keep using the dense matrix

            int i = 0;
            for(auto it = lagrangian_triplets.cbegin(); it != lagrangian_triplets.cend();
                    ++it) {
                iRow[it->row()]; jCol[it->col()];
                values[i] = h_lag_trig(it->row(), it->col());
            }
        }
        return true;
    }

    void IpoptAdapter::finalize_solution(SolverReturn status, Index n, const Number *x, const Number *z_L,
                                         const Number *z_U, Index m, const Number *g, const Number *lambda,
                                         Number obj_value, const IpoptData *ip_data,
                                         IpoptCalculatedQuantities *ip_cq) {
        std::cout << "FINITO PEPITO\n";
    }

}