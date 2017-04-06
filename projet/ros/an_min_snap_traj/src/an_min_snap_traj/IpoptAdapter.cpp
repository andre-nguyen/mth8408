//
// Created by andre on 4/6/17.
//

#include "an_min_snap_traj/IpoptAdapter.h"

using namespace Ipopt;
// Dont use Eigen namespace, conflicts with Index type

namespace an_min_snap_traj {
    IpoptAdapter::IpoptAdapter(Eigen::MatrixXd H, Eigen::MatrixXd Aeq, Eigen::VectorXd beq) :
            H_(H), Aeq_(Aeq), beq_(beq) {}

    IpoptAdapter::~IpoptAdapter() {}

    bool IpoptAdapter::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag,
                                    IndexStyleEnum &index_style) {
        n = H_.rows();
        m = beq_.rows();

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
    }

    bool IpoptAdapter::get_starting_point(Index n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U,
                                          Index m, bool init_lambda, Number *lambda) {

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
    }

    bool IpoptAdapter::eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) {
        assert(n == H_.rows());
        // Copy x
        Eigen::VectorXd c(n);
        for(Index i = 0; i < n; ++i) {
            c(i) = x[i];
        }

        // Gradient of f = 0.5 cHc'
        // 0.5 H'c
        Eigen::VectorXd result = 0.5 * H_.transpose() * c;
        for(int i = 0; i < n; ++i) {
            grad_f[i] = result(i);
        }
    }

    bool IpoptAdapter::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {

    }

    bool IpoptAdapter::eval_jac_g(Index n, const Number *x, bool new_x, Index m, Index nele_jac, Index *iRow,
                                  Index *jCol, Number *values) {

    }

    bool IpoptAdapter::eval_h(Index n, const Number *x, bool new_x, Number obj_factor, Index m, const Number *lambda,
                              bool new_lambda, Index nele_hess, Index *iRow, Index *jCol, Number *values) {

    }

    void IpoptAdapter::finalize_solution(SolverReturn status, Index n, const Number *x, const Number *z_L,
                                         const Number *z_U, Index m, const Number *g, const Number *lambda,
                                         Number obj_value, const IpoptData *ip_data,
                                         IpoptCalculatedQuantities *ip_cq) {}

}