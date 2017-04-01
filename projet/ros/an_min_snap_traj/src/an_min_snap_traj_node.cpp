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
    TrajectoryConstraint tc0(0, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());
    Vector3d wp1, wp2, wp3, wp4;
    wp1 << 0, 0, 1.5;
    wp2 << 1, 0, 1.5;
    wp3 << 1, 2, 1.5;
    wp4 << 0, 2, 1.5;
    TrajectoryConstraint tc1(1, wp1);
    TrajectoryConstraint tc2(2, wp2);
    TrajectoryConstraint tc3(3, wp3);
    TrajectoryConstraint tc4(4, wp4, Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());

    TrajectoryGenerator tg;
    tg.addConstraint(tc0);
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
    return 0;
}