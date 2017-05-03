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

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include <an_min_snap_traj/qp_solvers/IpoptAdapter.hpp>
#include <an_min_snap_traj/nlp_solvers/MyNLP.hpp>

using namespace an_min_snap_traj;
using namespace Eigen;

TEST(ipoptqp, LeastSquares)
{
    Vector2d solution(10.0, 8.0);
    MatrixXd Q(3,3);
    Q <<    6.0, 2.0, 1.0,
            2.0, 5.0, 2.0,
            1.0, 2.0, 4.0;
    VectorXd c(3);
    c << -8.0, -3.0, -3.0;
    MatrixXd Aeq(2,3);
    Aeq <<  1.0, 0.0, 1.0,
            0.0, 1.0, 1.0;
    VectorXd beq(2);
    beq <<  3.0, 0.0;
    VectorXd lb(3), ub(3);
    lb.fill(IpoptAdapter::MINUS_INFINITY);
    ub.fill(IpoptAdapter::PLUS_INFINITY);
    IpoptAdapter* ia = new IpoptAdapter(Q, c, Aeq, beq, lb, ub);
    Ipopt::SmartPtr<Ipopt::TNLP> qp = ia;
    Ipopt::SmartPtr<IpoptApplication> app =
            IpoptApplicationFactory();

    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("jac_c_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    //app->Options()->SetStringValue("linear_solver", "BLAS");
    app->Options()->SetStringValue("mehrotra_algorithm", "yes");
    app->Options()->SetIntegerValue("print_level", 5);
    app->Options()->SetIntegerValue("max_iter", 200);
    app->Options()->SetNumericValue("tol", 1e-8);
    if (status != Solve_Succeeded) {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    }
    status = app->OptimizeTNLP(qp);

    if(status == Ipopt::Solve_Succeeded){
        Ipopt::Index iter_count = app->Statistics()->IterationCount();
        std::cout << std::endl << std::endl << "*** The problem solved in "
                  << iter_count << " iterations!" << std::endl;

        Number final_obj = app->Statistics()->FinalObjective();
        std::cout << std::endl << std::endl
                  << "*** The final value of the objective function is "
                  << final_obj << '.' << std::endl;

    }
    assert(status == Solve_Succeeded);
}

TEST(ipoptqp, NLP )
{
    // Create an instance of your nlp...
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new MyNLP();

    // Create an instance of the IpoptApplication
    //
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    }

    status = app->OptimizeTNLP(mynlp);

    if (status == Ipopt::Solve_Succeeded) {
        // Retrieve some statistics about the solve
        Ipopt::Index iter_count = app->Statistics()->IterationCount();
        std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

        Number final_obj = app->Statistics()->FinalObjective();
        std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
    }

}