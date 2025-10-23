#ifndef KDL_SOLVER_H
#define KDL_SOLVER_H

#include <string>
#include <cfloat>
#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "std_msgs/msg/string.hpp"

class KDLSolver {
 private:
    std::shared_ptr<rclcpp::Node> n;
    std::string robot_desc_string;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    std::string arm_id_;
    std::string base_link_;
    std::string ee_link_;
    KDL::Tree kdlTree;
    std::unique_ptr<KDL::Chain[]> kdlChainsControlPoints;
    std::unique_ptr<KDL::Chain[]> kdlChainsJoints;
    int controlPointCount{0};

public:
    struct closest_point
    {
        int segmentId;
        Eigen::Vector3d segmentPointA;
        Eigen::Vector3d segmentPointB;
        double t;
        float distance_to_obs = FLT_MAX;
        Eigen::Vector3d control_point;
    };

    KDLSolver();
    // Initialize from ROS 2 node and build the KDL tree; returns true on success
    bool initialize(const std::shared_ptr<rclcpp::Node> & node, const std::string & arm_id = "fr3");
    Eigen::MatrixXd computeJacobian(std::string controlPointName, Eigen::VectorXd q);
    Eigen::MatrixXd computeJacobian2(KDLSolver::closest_point& controlPoint, Eigen::VectorXd& q);
    Eigen::MatrixXd forwardKinematicsJoints(const Eigen::VectorXd & q);
    Eigen::Vector3d forwardKinematicsControlPoints(std::string controlPointName, Eigen::VectorXd q);
    Eigen::Vector3d forwardKinematics(KDLSolver::closest_point& controlPoint, Eigen::VectorXd& q);
    int getNumberControlPoints();
    std::string getEELink() const;


};

#endif  // KDL_SOLVER_H
