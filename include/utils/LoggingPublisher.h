/*! @file LogginPublisher.h
    @brief Publishes external data to ROS topics.
*/

#ifndef LOGGING_PUBLISHER_H
#define LOGGING_PUBLISHER_H

#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "KDLSolver.h"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hiro_collision_avoidance_msgs/msg/control_point_constraints.hpp"
#include "hiro_collision_avoidance_msgs/msg/float_stamped.hpp"
#include "hiro_collision_avoidance_msgs/msg/manipulability_metrics.hpp"
#include "hiro_collision_avoidance_msgs/msg/point_array_stamped.hpp"
#include "hiro_collision_avoidance_msgs/msg/repulsive_vector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

/**
 * @class The following class is used to extract information from other classes.
 * That information is then published to a ROS topic. When running an
 * experiment, you can save all the output to a rosbag and then replay the data.
 */

class LoggingPublisher {
 public:
  LoggingPublisher();
  ~LoggingPublisher();
  void publishEE_Error(const double ee_error);
  void publishEEPath(const Eigen::Vector3d ee_path);
  void publishEETrajectory(const Eigen::Vector3d ee_path, const double time);
  void publishRepulsiveVector(const Eigen::Vector3d& vec,
                              const double& minDistance);
  void publishControlPointConstrains(
      const Eigen::VectorXd& boundlower, const Eigen::VectorXd& boundupper,
      const std::vector<Eigen::VectorXd>& boundlower_candidates,
      const std::vector<Eigen::VectorXd>& boundupper_candidates);
  void publishControlPoints(
      const std::vector<KDLSolver::closest_point>& controlPoints);
  void addEEasControlPoint(
      const Eigen::MatrixXd& J, const Eigen::MatrixXd& fkControlPointsAtq,
      std::vector<Eigen::MatrixXd>& controlPointsJc,
      std::vector<KDLSolver::closest_point>& controlPoints);
  void publishManipulabilityAnalysis(
      const std::vector<Eigen::MatrixXd>& Jc,
      const std::vector<KDLSolver::closest_point>& controlPoints,
      const Eigen::VectorXd& jointVelocityMax);

  void publishArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const int& id);

 private:
  // ROS-related instances
  std::shared_ptr<rclcpp::Node> nh_;
  int seq_val_;

  // publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr eeErrorPublisher_;
  rclcpp::Publisher<hiro_collision_avoidance_msgs::msg::FloatStamped>::SharedPtr eeErrorStampedPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr eePathPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr eeTrajectoryPublisher_;

  rclcpp::Publisher<hiro_collision_avoidance_msgs::msg::RepulsiveVector>::SharedPtr repulsiveVectorPub_;
  rclcpp::Publisher<hiro_collision_avoidance_msgs::msg::ControlPointConstraints>::SharedPtr controlPointConstraintsPub_;
  rclcpp::Publisher<hiro_collision_avoidance_msgs::msg::PointArrayStamped>::SharedPtr controlPointsPub_;
  rclcpp::Publisher<hiro_collision_avoidance_msgs::msg::ManipulabilityMetrics>::SharedPtr manipulabilityMetricsPub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arrowPub_;

  // messages
  std_msgs::msg::Float64 ee_error_msg_;
  hiro_collision_avoidance_msgs::msg::FloatStamped ee_error_msg_stamped_;
  std_msgs::msg::Float64MultiArray ee_path_msgarray_;
  std_msgs::msg::Float64MultiArray ee_traj_msgarray_;
};

#endif  // LOGGING_PUBLISHER_H
