#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H

#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "Eigen/Dense"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointVelocityController
{
    public:
        JointVelocityController();
        JointVelocityController(bool isSim);
        ~JointVelocityController();
        void sendVelocities(const Eigen::VectorXd velocities);

    private:
        // variable
        bool isSim;
        // ROS instances
        std::shared_ptr<rclcpp::Node> n;
        std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> velocityPublishers;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realVelocityPublisher;
        std_msgs::msg::Float64 msg;
        std_msgs::msg::Float64MultiArray msgarray;

        // functions
        void _switchController(bool isSim);
        void _prepareSimVelocityPublisher();
        void _prepareRealVelocityPublisher();
        void _sendSimVelocities(const Eigen::VectorXd velocities);
        void _sendRealVelocities(const Eigen::VectorXd velocities);
};

#endif // JOINT_VELOCITY_CONTROLLER_H
