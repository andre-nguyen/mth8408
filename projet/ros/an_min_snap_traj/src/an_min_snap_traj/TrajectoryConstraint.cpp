//
// Created by andre on 3/31/17.
//

#include <cmath>
#include <limits>

#include "an_min_snap_traj/TrajectoryConstraint.hpp"

namespace an_min_snap_traj {
    TrajectoryConstraint::TrajectoryConstraint(double time) {
        // Set all constraints to unconstrained by default
        double d = NAN;
        constraints_[Derivative::DER_POSITION].fill(d);
        constraints_[Derivative::DER_VELOCITY].fill(d);
        constraints_[Derivative::DER_ACCELERATION].fill(d);
        constraints_[Derivative::DER_JERK].fill(d);
        constraints_[Derivative::DER_SNAP].fill(d);
        time_ = time;
    }

    TrajectoryConstraint::TrajectoryConstraint(double time, Vector3d position) :
            TrajectoryConstraint(time) {
        constraints_[Derivative::DER_POSITION] = position;
    }

    TrajectoryConstraint::TrajectoryConstraint(double time, Vector3d position, Vector3d velocity) :
        TrajectoryConstraint(time, position) {
        constraints_[Derivative::DER_VELOCITY] = velocity;
    }

    TrajectoryConstraint::TrajectoryConstraint(double time, Vector3d position, Vector3d velocity,
                                               Vector3d acceleration) :
        TrajectoryConstraint(time, position, velocity) {
        constraints_[Derivative::DER_ACCELERATION] = acceleration;
    }

    TrajectoryConstraint::TrajectoryConstraint(double time, Vector3d position, Vector3d velocity,
                                               Vector3d acceleration, Vector3d jerk) :
        TrajectoryConstraint(time, position, velocity, acceleration) {
        constraints_[Derivative::DER_JERK] = jerk;
    }

    TrajectoryConstraint::TrajectoryConstraint(double time, Vector3d position, Vector3d velocity,
                                               Vector3d acceleration, Vector3d jerk,
                                               Vector3d snap) :
        TrajectoryConstraint(time, position, velocity, acceleration, jerk) {
        constraints_[Derivative::DER_SNAP] = snap;
    }

    Vector3d TrajectoryConstraint::getPosition() const{
        return constraints_[Derivative::DER_POSITION];
    }
    Vector3d TrajectoryConstraint::getVelocity() const{
        return constraints_[Derivative::DER_VELOCITY];
    }
    Vector3d TrajectoryConstraint::getAcceleration() const {
        return constraints_[Derivative::DER_ACCELERATION];
    }
    Vector3d TrajectoryConstraint::getJerk() const{
        return constraints_[Derivative::DER_JERK];
    }
    Vector3d TrajectoryConstraint::getSnap() const{
        return constraints_[Derivative::DER_SNAP];
    }
    Vector3d TrajectoryConstraint::getConstraint(int derivative) const {
        return constraints_[derivative];
    }

    double TrajectoryConstraint::getPosition(int dim) const{
        return constraints_[Derivative::DER_POSITION](dim);
    }
    double TrajectoryConstraint::getVelocity(int dim) const{
        return constraints_[Derivative::DER_VELOCITY](dim);
    }
    double TrajectoryConstraint::getAcceleration(int dim) const {
        return constraints_[Derivative::DER_ACCELERATION](dim);
    }
    double TrajectoryConstraint::getJerk(int dim) const{
        return constraints_[Derivative::DER_JERK](dim);
    }
    double TrajectoryConstraint::getSnap(int dim) const{
        return constraints_[Derivative::DER_SNAP](dim);
    }
    double TrajectoryConstraint::getTime() const{
        return time_;
    }
    int TrajectoryConstraint::getConstraintCount(int dim) const {
        int count = 0;
        for(int i = 0; i < Derivative::DER_COUNT; ++i) {
            if(isConstrained(i, dim)) count++;
        }
        return count;
    }

    void TrajectoryConstraint::setPosition(const Vector3d pos){
        constraints_[Derivative::DER_POSITION] = pos;
    }
    void TrajectoryConstraint::setVelocity(const Vector3d vel){
        constraints_[Derivative::DER_VELOCITY] = vel;
    }
    void TrajectoryConstraint::setAcceleration(const Vector3d acc) {
        constraints_[Derivative::DER_ACCELERATION] = acc;
    }
    void TrajectoryConstraint::setJerk(const Vector3d jerk){
        constraints_[Derivative::DER_JERK] = jerk;
    }
    void TrajectoryConstraint::setSnap(const Vector3d snap){
        constraints_[Derivative::DER_SNAP] = snap;
    }
    void TrajectoryConstraint::setConstraint(const int derivative, const Vector3d constraint) {
        constraints_[derivative] = constraint;
    }

    bool TrajectoryConstraint::isConstrained(int derivative, int dimension) const {
        double d = std::numeric_limits<double>::max();
        if(derivative < Derivative::DER_COUNT) {
            return !std::isnan(constraints_[derivative](dimension));
        } else {
            return false;
        }
    }
}
