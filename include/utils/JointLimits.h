#ifndef JOINT_LIMITS_H
#define JOINT_LIMITS_H

#include <iostream>
#include <vector>

#include "Eigen/Dense"


class JointLimits
{
    public:

        JointLimits();
        JointLimits(Eigen::VectorXd jointLimitsMin,  Eigen::VectorXd jointLimitsMax,
                                 Eigen::VectorXd jointMiddleValues,  Eigen::VectorXd jointRanges,
                                 Eigen::VectorXd jointVelocityMax,  Eigen::VectorXd jointAccelerationMax);

        JointLimits(const JointLimits&);
        Eigen::VectorXd jointLimitsMin{7};
        Eigen::VectorXd jointLimitsMax{7};
        Eigen::VectorXd jointMiddleValues{7};
        Eigen::VectorXd jointRanges{7};
        Eigen::VectorXd jointVelocityMax{7};
        Eigen::VectorXd jointAccelerationMax{7};

    private:
};

#endif // LOGGING_PUBLISHER_H
