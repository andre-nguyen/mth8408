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

    void TrajectoryConstraint::setTime(const double time) {
        time_ = time;
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
