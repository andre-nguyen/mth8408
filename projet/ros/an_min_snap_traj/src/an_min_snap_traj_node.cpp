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

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

#include <an_min_snap_traj/TrajectoryGenerator.hpp>
#include <an_min_snap_traj/TrajectoryConstraint.hpp>
#include <an_min_snap_traj/TimeAllocationOpt.hpp>


using namespace an_min_snap_traj;
using namespace Eigen;

int main(int argc, char** argv) {
    Vector3d wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8, wp9;
    wp1 << 0, 0, 1;
    wp2 << 2, -2, 1.5;
    wp3 << 4, 0, 2;
    wp4 << 2, 2, 1.5;
    wp5 << 0, 0, 1;
    wp6 << -2, -2, 1.5;
    wp7 << -4, 0, 2;
    wp8 << -2, 2, 1.5;
    wp9 << 0, 0, 1;

    TrajectoryConstraint tc1(0, wp1, Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());
    TrajectoryConstraint tc2(1, wp2);
    TrajectoryConstraint tc3(2, wp3);
    TrajectoryConstraint tc4(3, wp4);
    TrajectoryConstraint tc5(4, wp5);
    TrajectoryConstraint tc6(5, wp6);
    TrajectoryConstraint tc7(6, wp7);
    TrajectoryConstraint tc8(7, wp8);
    TrajectoryConstraint tc9(8, wp9, Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());

    TrajectoryGenerator *tg = new TrajectoryGenerator();
    tg->addConstraint(tc1);
    tg->addConstraint(tc2);
    tg->addConstraint(tc3);
    tg->addConstraint(tc4);
    tg->addConstraint(tc5);
    tg->addConstraint(tc6);
    tg->addConstraint(tc7);
    tg->addConstraint(tc8);
    tg->addConstraint(tc9);

    TimeAllocationOpt taopt(tg, TrajectoryGenerator::Solver::OOQP);
    auto started = std::chrono::high_resolution_clock::now();
    try {
        bool result = taopt.optimize(TimeAllocationOpt::Solver::NLOPT_AUGLAG);
        std::cout << (result ? "success\n" : "fail\n");
    } catch (alglib::ap_error e) {
        std::cout << e.msg;
    }
    auto done = std::chrono::high_resolution_clock::now();
    long time = std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count();
    std::cout << "execution time " << time << " ms" << std::endl;
    std::cout << "obj val " << tg->getObjectiveFuncVal();

    delete tg;/*


    auto trajp = tg->discretizeSolution();
    auto trajv = tg->getDiscreteSolution(DER_VELOCITY);
    auto traja = tg->getDiscreteSolution(DER_ACCELERATION);

    ros::init(argc, argv, "an_traj_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("firefly/command/pose", 10);
    ros::Publisher pub_traj = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("firefly/command/trajectory", 10);

    trajectory_msgs::MultiDOFJointTrajectory ros_traj;
    for(int i = 0; i < trajp.size(); ++i) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint p;
        geometry_msgs::Transform t;
        t.translation.x = trajp[i](0);
        t.translation.y = trajp[i](1);
        t.translation.z = trajp[i](2);
        p.transforms.push_back(t);
        geometry_msgs::Twist v;
        v.linear.x = trajv[i](0);
        v.linear.y = trajv[i](1);
        v.linear.z = trajv[i](2);
        p.velocities.push_back(v);
        geometry_msgs::Twist a;
        a.linear.x = traja[i](0);
        a.linear.y = traja[i](1);
        a.linear.z = traja[i](2);
        p.accelerations.push_back(a);
        p.time_from_start.fromSec(i * 0.1);
        ros_traj.points.push_back(p);
    }

    ros::Rate r(1);
    while(pub_traj.getNumSubscribers() < 1)
        r.sleep();

    pub_traj.publish(ros_traj);
    ros::spin();
/*
    r = ros::Rate(10);
    for(int i = 0; i < trajp.size(); ++i) {
        geometry_msgs::PoseStamped p;
        p.pose.position.x = trajp[i](0);
        p.pose.position.y = trajp[i](1);
        p.pose.position.z = trajp[i](2);
        pub.publish(p);
        ros::spinOnce();
        r.sleep();
    }
*/
    return 0;
}
