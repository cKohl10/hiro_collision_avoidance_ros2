#include <string>
#include <vector>
#include <algorithm>
#include <deque>
#include <signal.h>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <mutex>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "JointVelocityController.h"
#include "QPAvoidance.h"


#include "Eigen/Dense"

#include "KDLSolver.h"
#include "HIROAvoidance.h"
#include "utils/JointLimits.h"

enum AvoidanceMode {noAvoidance, HIRO};
class CartesianPositionController {
 private:
    // Private Member Variables
    Eigen::VectorXd q, prevQDot, qDot{7};

    Eigen::VectorXd current_Qdot{7};
    Eigen::MatrixXd J, Jpinv, joint_positions;
    JointLimits jointLimits;
    std::deque<Eigen::VectorXd> prevControlOutputs;

    int maxPrevOutputsSize = 4;
    float current_vel_one_step = 0.0;
    bool isSim;
    AvoidanceMode avoidanceMode{noAvoidance};
    int numberControlPoints;
    double positionErrorThreshold{0.005}, pGain {2.5}, secondaryTaskGain{5.0};
    std::shared_ptr<rclcpp::Node> n;
    rclcpp::Rate rate{100.0};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriberJointStates;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriberControllerSwitch;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberObstaclePoints;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriberExternalCartesianWrench;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriberExternalCartesianWrenchRaw;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr test_pub;

    bool robotInContact = false;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ee_xyz_publisher;
    geometry_msgs::msg::Point ee_xyz;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::vector<Eigen::Vector3d> obstaclePositionVectors;
    std::unique_ptr<Eigen::Vector3d[]> controlPointPositionVectors;
    Eigen::Vector3d endEffectorPosition, positionError, desiredEEVelocity;
    Eigen::VectorXd endEffectorPose;
    std::vector<KDLSolver::closest_point> closestPointsOnRobot;
    KDLSolver kdlSolver;
    std::mutex mutex;
    std::string ee_link_name_;


    Eigen::Vector3d EEVelocity;

    //Helper Functions
    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr scan);
    void ExternalCartesianWrenchCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void ControllerSwitchCallback(const std_msgs::msg::String::SharedPtr msg);

    void ExternalCartesianWrenchRawCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void ObstaclePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void readControlPointPositions();
    void findClosestControlPointsToObstacles();
    Eigen::Vector3d getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t);
    Eigen::VectorXd EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity);
    Eigen::VectorXd EEVelocityToQDotRepulsive(Eigen::Vector3d desiredEEVelocity, std::vector<Eigen::Vector3d> obstaclePositionVectors);

    void setJointLimits(std::shared_ptr<rclcpp::Node> node_handle);
    void getControlPoints();
    void createRosPubsSubs();

    QPAvoidance qpAvoidance;
    HIROAvoidance hiroAvoidance;
    JointVelocityController jointVelocityController;
    Eigen::Vector3d goal;
    float cumulative_error;
    float derivative_error;

 public:

    // Default Constructor
    CartesianPositionController(bool isSim, std::shared_ptr<rclcpp::Node> node_handle);
    CartesianPositionController(bool isSim);

    AvoidanceMode getAvoidanceMode();
    void setAvoidanceMode(AvoidanceMode avoidanceModeName);

    void moveToPosition(Eigen::Vector3d position_vector);
    void moveToPositionOneStep(Eigen::Vector3d position_vector, bool isEnd=true);
    void moveToPositionOneStepSeq(const Eigen::Vector3d desiredPositionVector, bool isEnd, double ee_ro);
    void commandVelocityOneStep(Eigen::Vector3d desiredEEVelocityVector, Eigen::Vector3d desiredPositionVector);

    void setVelocitiesToZero();
    void moveToStart();
    void readEndEffectorPosition();
    Eigen::Vector3d getEEPosition();
    Eigen::Vector3d getEEVelocity();
    Eigen::Vector3d externalCartesianForce{3};
    Eigen::Vector3d externalCartesianWrench{3};


};
