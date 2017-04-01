//
// Created by andre on 3/31/17.
//

#ifndef TRAJECTORYCONSTRAINT_H
#define TRAJECTORYCONSTRAINT_H

#include <Eigen/Core>
#include "TrajectoryMath.hpp"

using namespace Eigen;

namespace an_min_snap_traj {
    class TrajectoryConstraint {
    public:
        /**
         * By default all constraints are set to unconstrained (i.e. std::numeric_limits<double>::max())
         */
        TrajectoryConstraint(double time);
        TrajectoryConstraint(double time, Vector3d position);
        TrajectoryConstraint(double time, Vector3d position, Vector3d velocity);
        TrajectoryConstraint(double time, Vector3d position, Vector3d velocity, Vector3d acceleration);
        TrajectoryConstraint(double time, Vector3d position, Vector3d velocity, Vector3d acceleration, Vector3d jerk);
        TrajectoryConstraint(double time, Vector3d position, Vector3d velocity, Vector3d acceleration, Vector3d jerk, Vector3d snap);

        Vector3d getPosition() const;
        Vector3d getVelocity() const;
        Vector3d getAcceleration() const;
        Vector3d getJerk() const;
        Vector3d getSnap() const;
        Vector3d getConstraint(int derivative) const;
        double getPosition(int dim) const;
        double getVelocity(int dim) const;
        double getAcceleration(int dim) const;
        double getJerk(int dim) const;
        double getSnap(int dim) const;
        double getTime() const;

        /**
         * Counts the number of constrained derivatives in a
         * certain dimension.
         * @param dim Dimension to check
         * @return
         */
        int getConstraintCount(int dim) const;

        void setPosition(const Vector3d pos);
        void setVelocity(const Vector3d vel);
        void setAcceleration(const Vector3d acc);
        void setJerk(const Vector3d jerk);
        void setSnap(const Vector3d snap);
        void setConstraint(const int derivative, const Vector3d constraint);
        void setTime(const double time);

        /**
         * Checks if the derivative of a certain dimension is constrained
         * @param derivative
         * @param dimension
         * @return
         */
        bool isConstrained(int derivative, int dimension) const;

    private:
        Vector3d constraints_[Derivative::DER_COUNT];
        double time_;
    };
}

#endif //TRAJECTORYCONSTRAINT_H
