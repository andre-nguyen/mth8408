//
// Created by andre on 3/31/17.
//

#include <iostream>
#include <Eigen/Dense>

#include <an_min_snap_traj/TrajectoryGenerator.hpp>
#include <an_min_snap_traj/TrajectoryConstraint.hpp>


using namespace an_min_snap_traj;
using namespace Eigen;

int main(int argc, char** argv) {
    Vector3d wp1, wp2, wp3, wp4;
    wp1 << 0, 0, 1.5;
    wp2 << 1, 0, 1.5;
    wp3 << 1, 2, 1.5;
    wp4 << 0, 2, 1.5;
    TrajectoryConstraint tc1(0, wp1);
    TrajectoryConstraint tc2(1, wp2);
    TrajectoryConstraint tc3(2, wp3);
    TrajectoryConstraint tc4(3, wp4, Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());

    TrajectoryGenerator tg;
    tg.addConstraint(tc1);
    tg.addConstraint(tc2);
    tg.addConstraint(tc3);
    tg.addConstraint(tc4);
    tg.buildProblem();
    std::cout << "Cost matrix: \n" << tg.getCostMatrix(0) << std::endl;
    std::cout << "Size \n" << tg.getCostMatrix(0).rows() << ' ' << tg.getCostMatrix(0).cols() << std::endl;
    std::cout << "Num constraints " << tg.getNumConstraints(0) << std::endl;
    auto constraints = tg.getConstraints();
    for(std::vector<TrajectoryConstraint>::const_iterator it = constraints.cbegin();
            it != constraints.cend(); it++) {
        for(int i = 0; i < Derivative::DER_COUNT; ++i) {
            std::cout << it->getConstraint(i)(0) << '\t';
        }
        std::cout << std::endl;
    }

    MatrixXd I(3,3);
    I << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    MatrixXd expected(3,3);
    expected << 3, 6, 9, 2, 5, 8, 1, 4, 7;
    std::cout << "Eye \n" << rot90(I) << std::endl;
    auto mat_answer = genCoefficientMatrix(6, 4);
    std::cout << "Coefficient matrix : \n" << mat_answer << std::endl;

    mat_answer = tg.getFixedConstraintMatrix(0);
    std::cout << "Constraints matrix X: \n" << mat_answer << std::endl;
    std::cout << "b vector fixed \n" << tg.getFixedConstraintVector(0) << std::endl;

    mat_answer = tg.getContinuityConstraintMatrix(0);
    std::cout << "Constraints continuit matrix X: \n" << mat_answer << std::endl;
    std::cout << "b vector continuit\n" << tg.getContinuityConstraintVector(0) << std::endl;


    tg.solveProblem(0, TrajectoryGenerator::Solver::OOQP);
    tg.solveProblem(1, TrajectoryGenerator::Solver::OOQP);
    tg.solveProblem(2, TrajectoryGenerator::Solver::OOQP);
    std::cout << "solution X\n" << tg.getSolution(0) << std::endl;
    std::cout << "solution Y\n" << tg.getSolution(1) << std::endl;
    std::cout << "solution Z\n" << tg.getSolution(2) << std::endl;
    return 0;
}