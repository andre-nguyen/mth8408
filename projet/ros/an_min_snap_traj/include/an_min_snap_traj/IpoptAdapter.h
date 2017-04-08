//
// Created by andre on 4/5/17.
//

#ifndef IPOPTADAPTER_H
#define IPOPTADAPTER_H

#include <Eigen/Core>

#include <IpTNLP.hpp>

using namespace Ipopt;

namespace an_min_snap_traj {
    class IpoptAdapter : public TNLP {
    public:
        IpoptAdapter(Eigen::MatrixXd H, Eigen::MatrixXd Aeq, Eigen::VectorXd beq);

        virtual ~IpoptAdapter();

        /** Method to return some info about the nlp */
        virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                                  Ipopt::Index& nnz_h_lag, IndexStyleEnum& Index_style);

        /** Method to return the bounds for my problem */
        virtual bool get_bounds_info(Ipopt::Index n, Number* x_l, Number* x_u,
                                     Ipopt::Index m, Number* g_l, Number* g_u);

        /** Method to return the starting point for the algorithm */
        virtual bool get_starting_point(Ipopt::Index n, bool init_x, Number* x,
                                        bool init_z, Number* z_L, Number* z_U,
                                        Ipopt::Index m, bool init_lambda,
                                        Number* lambda);

        /** Method to return the objective value */
        virtual bool eval_f(Ipopt::Index n, const Number* x, bool new_x, Number& obj_value);

        /** Method to return the gradient of the objective */
        virtual bool eval_grad_f(Ipopt::Index n, const Number* x, bool new_x, Number* grad_f);

        /** Method to return the constraint residuals */
        virtual bool eval_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Number* g);

        /** Method to return:
         *   1) The structure of the jacobian (if "values" is NULL)
         *   2) The values of the jacobian (if "values" is not NULL)
         */
        virtual bool eval_jac_g(Ipopt::Index n, const Number* x, bool new_x,
                                Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                                Number* values);

        /** Method to return:
         *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
         *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
         *
         * @param n           [in]  the number of variables in the problem (dimension of $ x$).
         * @param x           [in]  the values for the primal variables, $ x$, at which the Hessian is to be evaluated.
         * @param new_x       [in]  false if any evaluation method was previously called with the same values in x, true otherwise.
         * @param obj_factor  [in]  factor in front of the objective term in the Hessian, $ \sigma_f$.
         * @param m           [in]  the number of constraints in the problem (dimension of $ g(x)$).
         * @param lambda      [in]  the values for the constraint multipliers,  $ \lambda$, at which the Hessian is to be evaluated.
         * @param new_lambda  [in]  false if any evaluation method was previously called with the same values in lambda, true otherwise.
         * @param nele_hess   [in]  the number of nonzero elements in the Hessian (dimension of iRow, jCol, and values).
         * @param iRow        [out] the row indices of entries in the Hessian.
         * @param jCol        [out] the column indices of entries in the Hessian.
         * @param values      [out] the values of the entries in the Hessian.
         * @return
         */
        virtual bool eval_h(Ipopt::Index n, const Number* x, bool new_x,
                            Number obj_factor, Ipopt::Index m, const Number* lambda,
                            bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                            Ipopt::Index* jCol, Number* values);

        virtual void finalize_solution(SolverReturn status,
                                       Ipopt::Index n, const Number* x, const Number* z_L, const Number* z_U,
                                       Ipopt::Index m, const Number* g, const Number* lambda,
                                       Number obj_value,
                                       const IpoptData* ip_data,
                                       IpoptCalculatedQuantities* ip_cq);
    private:
        IpoptAdapter(const IpoptAdapter&);
        IpoptAdapter& operator=(const IpoptAdapter&);

        Eigen::MatrixXd H_, Aeq_;
        Eigen::VectorXd beq_;
        std::vector<Eigen::Triplet<double>> lagrangian_triplets; // holds all the non zero triplets
        int nnz_jac_g_, nnz_h_lag_;
        

    };
}

#endif //IPOPTADAPTER_H
