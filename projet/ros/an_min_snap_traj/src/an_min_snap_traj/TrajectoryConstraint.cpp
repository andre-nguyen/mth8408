//
// Created by andre on 3/31/17.
//

#include <limits>

#include "an_min_snap_traj/TrajectoryConstraint.hpp"

namespace an_min_snap_traj {
    TrajectoryConstraint::TrajectoryConstraint() {
        // Set all constraints to unconstrained by default
        double d = std::numeric_limits<double>::max();
        position_ << d, d, d;
        velocity_ << d, d, d;
        jerk_ << d, d, d;
        snap_ << d, d, d;
        time_ = 0;
    }

    TrajectoryConstraint::TrajectoryConstraint(Vector3d position) : TrajectoryConstraint() {
        position_ = position;
    }

    TrajectoryConstraint::TrajectoryConstraint(Vector3d position, Vector3d velocity) :
        TrajectoryConstraint(position) {
        velocity_ = velocity;
    }

    TrajectoryConstraint::TrajectoryConstraint(Vector3d position, Vector3d velocity,
                                               Vector3d acceleration) :
        TrajectoryConstraint(position, velocity) {
        acceleration_ = acceleration;
    }

    TrajectoryConstraint::TrajectoryConstraint(Vector3d position, Vector3d velocity,
                                               Vector3d acceleration, Vector3d jerk) :
        TrajectoryConstraint(position, velocity, acceleration) {
        jerk_ = jerk;
    }

    TrajectoryConstraint::TrajectoryConstraint(Vector3d position, Vector3d velocity,
                                               Vector3d acceleration, Vector3d jerk,
                                               Vector3d snap) :
        TrajectoryConstraint(position, velocity, acceleration, jerk) {
        snap_ = snap;
    }

    Vector3d TrajectoryConstraint::getPosition() const{
        return position_;
    }
    Vector3d TrajectoryConstraint::getVelocity() const{
        return velocity_;
    }
    Vector3d TrajectoryConstraint::getAcceleration() const {
        return acceleration_;
    }
    Vector3d TrajectoryConstraint::getJerk() const{
        return jerk_;
    }
    Vector3d TrajectoryConstraint::getSnap() const{
        return snap_;
    }

    double TrajectoryConstraint::getPosition(int dim) const{
        return position_(dim);
    }
    double TrajectoryConstraint::getVelocity(int dim) const{
        return velocity_(dim);
    }
    double TrajectoryConstraint::getAcceleration(int dim) const {
        return acceleration_(dim);
    }
    double TrajectoryConstraint::getJerk(int dim) const{
        return jerk_(dim);
    }
    double TrajectoryConstraint::getSnap(int dim) const{
        return snap_(dim);
    }
    double TrajectoryConstraint::getTime() const{
        return time_;
    }

    void TrajectoryConstraint::setPosition(const Vector3d pos){
        position_ = pos;
    }
    void TrajectoryConstraint::setVelocity(const Vector3d vel){
        velocity_ = vel;
    }
    void TrajectoryConstraint::setAcceleration(const Vector3d acc) {
        acceleration_ = acc;
    }
    void TrajectoryConstraint::setJerk(const Vector3d jerk){
        jerk_ = jerk;
    }
    void TrajectoryConstraint::setSnap(const Vector3d snap){
        snap_ = snap;
    }

    bool TrajectoryConstraint::isConstrained(int derivative, int dimension) {
        double d = std::numeric_limits<double>::max();
        switch(derivative) {
            case 0:
                return position_(dimension) == d;
            case 1:
                return velocity_(dimension) == d;
            case 2:
                return acceleration_(dimension) == d;
            case 3:
                return jerk_(dimension) == d;
            case 4:
                return snap_(dimension) == d;
            default:
                return false;
        }
    }
}
