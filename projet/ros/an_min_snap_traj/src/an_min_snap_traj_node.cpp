//
// Created by andre on 3/31/17.
//

#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

#include <an_min_snap_traj/TrajectoryGenerator.hpp>
#include <an_min_snap_traj/TrajectoryConstraint.hpp>

using namespace an_min_snap_traj;
using namespace Eigen;

int main(int argc, char** argv) {
    Vector3d wp1, wp2, wp3, wp4;
    wp1 << 0, 0, 1.5;
    wp2 << 5, 0, 3;
    wp3 << 5, 10, 3;
    wp4 << 0, 10, 1.5;
    TrajectoryConstraint tc1(0, wp1, Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());
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

    tg.solveProblem(TrajectoryGenerator::Solver::OOQP);
    auto trajp = tg.discretizeSolution();
    auto trajv = tg.getDiscreteSolution(DER_VELOCITY);
    auto traja = tg.getDiscreteSolution(DER_ACCELERATION);

    ros::init(argc, argv, "an_traj_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("firefly/command/pose", 10);
/*
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
        p.time_from_start.fromSec(0.1);
        ros_traj.points.push_back(p);
    }*/

    ros::Rate r(1);
    while(pub.getNumSubscribers() < 1)
        r.sleep();

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

    return 0;
}
