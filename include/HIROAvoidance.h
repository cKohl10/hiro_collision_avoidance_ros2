#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "KDLSolver.h"
#include "CollisionAvoidanceBase.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <math.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "utils/JointLimits.h"
#include "rclcpp/rclcpp.hpp"

#include <zScore.h>

class HIROAvoidance: public CollisionAvoidanceBase
{
private:
    // Private Variables
    double _dampingFactor0{0.1};
    double _omega0{0.001};
    double dampingFactor0{0.1}, omega0{0.001};
    double computeDampingFactor(double omega);
    double computeDampingFactorV2(double b_0, double a, Eigen::MatrixXd jacobian);
    Eigen::MatrixXd cartesianComplianceMatrix;
    bool isCloseToZero(double value, double threshold);
    int obstacle_life;
    double force_memory;
    double slow_down_memory_y;
    double life_memory_y;
    int hit_life;
    double force_hit;
    Eigen::Vector3d ext_vel;

    std::string ee_link_name;
    std::string base_link_name;


    JointLimits _jointLimits;


    // Private Functions
    double computebvalue(double distanceNorm);
    Eigen::VectorXd alter_ee_path();
    Eigen::VectorXd compliance_gains;
    Eigen::Vector3d forceBasedRepulsiveVelocity;


    Eigen::VectorXd gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, KDLSolver::closest_point closestPoint, Eigen::VectorXd q);

    void getVelocityLimits(Eigen::VectorXd& lower_bound, Eigen::VectorXd& upper_bound,
                                      Eigen::VectorXd& q, rclcpp::Rate& r);

    Eigen::MatrixXd getHMatrix(Eigen::VectorXd& q, Eigen::Vector3d& xDot, bool drop_goal);
    Eigen::VectorXd getfVector(Eigen::VectorXd& q, Eigen::Vector3d& xDot, bool drop_goal);
    bool drop_goal = false;


    Eigen::VectorXd q_dot;

    void resetComputationObjects();
    void computeObstacleThresholdReduction();

    // Refactor, these values are calculated inside this function
    void alterEEVelocityFromObstacles(Eigen::Vector3d & xDot, std::vector<Eigen::Vector3d>& obstaclePositionVectors, Eigen::Vector3d& ee_pos);
    bool new_obs = false;
    std::vector<double> x_dists_pos;
    std::vector<double> y_dists_pos;
    std::vector<double> z_dists_pos;
    std::vector<double> x_dists_neg;
    std::vector<double> y_dists_neg;
    std::vector<double> z_dists_neg;

    void updateExistingObstacleEEForce(Eigen::Vector3d & xDot);
    float max_life = 200.0;

    //Variables used for collision thresholding
    double min_newton_dif = 20;
    double max_std_dev_dif = 4;
    double max_std_dev = 3;
    double max_dist_dif = 4;

    double dist_max_reduction = 4.5;
    double max_dist = 0.3;
    double min_dist = 0.05;

    double min_x_dist = 0;
    double max_x_dist = 0;
    double x_dist_pos_dif = 0;
    double x_dist_neg_dif = 0;

    double min_y_dist = 0;
    double max_y_dist = 0;
    double y_dist_pos_dif = 0;
    double y_dist_neg_dif = 0;

    double min_z_dist = 0;
    double max_z_dist = 0;
    double z_dist_pos_dif = 0;
    double z_dist_neg_dif = 0;

    double x_mean;
    double y_mean;
    double z_mean;

    double x_std_dev;
    double y_std_dev;
    double z_std_dev;

    double added_x_std_dev;
    double added_y_std_dev;
    double added_z_std_dev;

    double x_neg_thresh;
    double x_pos_thresh;
    double y_neg_thresh;
    double y_pos_thresh;
    double z_neg_thresh;
    double z_pos_thresh;

    bool x_contact = false;
    bool y_contact = false;
    bool z_contact = false;

    void checkForContact(Eigen::Vector3d & externalWrench);
    void applyContactBehavior(Eigen::Vector3d & externalWrench);
    void updateExistingObstacleThresholdReduction();
    void updateExistingContactForce(Eigen::Vector3d & xDot, bool &inContact);

    std::vector<std::tuple<Eigen::Vector3d, KDLSolver::closest_point>> objectHistory;


    // zScores
    zScore signal_parser_x;
    zScore signal_parser_y;
    zScore signal_parser_z;

    zScore signal_parser_x_upper;
    zScore signal_parser_x_lower;

    zScore signal_parser_y_upper;
    zScore signal_parser_y_lower;

    zScore signal_parser_z_upper;
    zScore signal_parser_z_lower;



public:
    HIROAvoidance();
    HIROAvoidance(JointLimits jointLimits, std::shared_ptr<rclcpp::Node> node_handle);
    void setCartesianComplianceGains(Eigen::VectorXd gains);
    Eigen::VectorXd computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d xDot,
                                           std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                           const std::vector<KDLSolver::closest_point>& constClosestPoints,
                                           rclcpp::Rate& r, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_goal,
                                           Eigen::MatrixXd& jointPositions);
    Eigen::VectorXd computeJointVelocitiesWithExtCartForce(Eigen::VectorXd& q, Eigen::Vector3d xDot,
                                                    Eigen::Vector3d & externalCartesianForce,
                                                    Eigen::Vector3d & externalWrench,
                                                    std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                    std::vector<KDLSolver::closest_point>& closestPoints,
                                                    rclcpp::Rate& r, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_goal,
                                                    Eigen::Vector3d EEVelocity,
                                                    bool& inContact);

};