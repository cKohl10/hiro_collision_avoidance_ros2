#include "utils/JointLimits.h"


JointLimits::JointLimits( Eigen::VectorXd jointLimitsMin, Eigen::VectorXd jointLimitsMax,
                          Eigen::VectorXd jointMiddleValues, Eigen::VectorXd jointRanges,
                          Eigen::VectorXd jointVelocityMax, Eigen::VectorXd jointAccelerationMax, Eigen::VectorXd jointJerkMax) {

    this->jointLimitsMin = jointLimitsMin;
    this->jointLimitsMax = jointLimitsMax;
    this->jointMiddleValues = jointMiddleValues;
    this->jointRanges = jointRanges;
    this->jointVelocityMax = jointVelocityMax;
    this->jointAccelerationMax = jointAccelerationMax;
    this->jointJerkMax = jointJerkMax;
}


JointLimits::JointLimits(const JointLimits& other){

    this->jointLimitsMin = other.jointLimitsMin;
    this->jointLimitsMax = other.jointLimitsMax;
    this->jointMiddleValues = other.jointMiddleValues;
    this->jointRanges = other.jointRanges;
    this->jointVelocityMax = other.jointVelocityMax;
    this->jointAccelerationMax = other.jointAccelerationMax;
    this->jointJerkMax = other.jointJerkMax;
}
JointLimits::JointLimits(){}