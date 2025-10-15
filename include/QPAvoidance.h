#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "KDLSolver.h"
#include "CollisionAvoidanceBase.h"


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <math.h>
#include "utils/JointLimits.h"


class QPAvoidance: public CollisionAvoidanceBase
{
private:
    // Given from equation 18
    // TODO: place all constants in the header file
    double dampingFactor0{0.1}, omega0{0.001};
    double computeDampingFactor(double omega);
    JointLimits _jointLimits;

    Eigen::VectorXd gradientOfDistanceNorm(
                    Eigen::Vector3d obstaclePositionVector, 
                    KDLSolver::closest_point closestPoint, Eigen::VectorXd q);

    double computebvalue(double distanceNorm);
    void calculateJointVelocityBounds(
        Eigen::VectorXd &bl, Eigen::VectorXd&bu, 
        const Eigen::VectorXd& q, const rclcpp::Rate& r);
    void setupOptimizationMainTask(
        Eigen::MatrixXd& H, Eigen::VectorXd& f, 
        const Eigen::VectorXd& q);
    void setupOptimizationMainTask(
        Eigen::MatrixXd& H, Eigen::VectorXd& f, 
        const Eigen::VectorXd& q, const Eigen::Vector3d& xDot);
    int calculateOptimizationRowValues(
        Eigen::MatrixXd& A, Eigen::VectorXd& b, 
        const  std::vector<Eigen::Vector3d>& obstaclePositionVectors, 
        const std::vector<KDLSolver::closest_point>& closestPoints, 
        const Eigen::VectorXd& q);


public:
    QPAvoidance();
    QPAvoidance(JointLimits jointLimits);
    Eigen::VectorXd computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d& xDot,
                                           std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                           std::vector<KDLSolver::closest_point>& closestPoints,
                                           rclcpp::Rate& r);

};