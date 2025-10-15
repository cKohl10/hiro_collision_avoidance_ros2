#include "JointVelocityController.h"

// extra layer on top of joint velocity controller:
// publishes to a topic that the main velocity controller takes in.

JointVelocityController::JointVelocityController() {

    this->isSim = true;


    // Switches to a proper controller depending on sim / real robot
    _switchController(isSim);

    // Set up Velocity Publishers for Sim and Real
    // Sim
    _prepareSimVelocityPublisher();
    // Real
    _prepareRealVelocityPublisher();

    msgarray.data.resize(7);
}

/*

constructor for the joint velocity controller.

*/

JointVelocityController::JointVelocityController(bool isSim=true) {
    this->isSim = isSim;

    // Switches to a proper controller depending on sim / real robot
    _switchController(isSim);

    // Set up Velocity Publishers for Sim and Real
    // Sim
    _prepareSimVelocityPublisher();
    // Real
    _prepareRealVelocityPublisher();

    msgarray.data.resize(7);
}


JointVelocityController::~JointVelocityController() {}

/*
    Switches to a proper controller depending on sim / real robot
*/
void JointVelocityController::_switchController(bool isSim){
    // Append 7 different joint controllers
    std::vector<std::string> velocityControllerNames;
    std::vector<std::string> positionControllerNames;
    if (isSim) {
        // Prepare Empty Vectors
        velocityControllerNames.reserve(7);
        positionControllerNames.reserve(7);
        // Append controllers to the vectors
        for (int i = 0; i < 7; i++) {
            std::string velocityControllerName = "panda_joint" + std::to_string(i+1) + "_velocity_controller";
            std::string positionControllerName = "panda_joint" + std::to_string(i+1) + "_position_controller";
            velocityControllerNames.push_back(velocityControllerName);
            positionControllerNames.push_back(positionControllerName);
        }
        positionControllerNames.push_back("panda_joint_trajectory_controller");
    // Only append 1 joint controller
    } else {
        // Prepare Empty Vectors
        velocityControllerNames.reserve(1);
        positionControllerNames.reserve(1);
        // Append controllers to the vectors
        for (int i = 0; i < 1; i++) {
            velocityControllerNames.push_back("panda_joint_velocity_contact_controller");
            positionControllerNames.push_back("panda_joint_position_controller");
        }
    }

        // Prepare a ROS Service to switch controllers
        auto switch_controller = n->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
        // Msg
        auto srv = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        srv->start_controllers = velocityControllerNames;
        srv->stop_controllers = positionControllerNames;
        srv->strictness = 1;
        srv->start_asap = true;
        srv->timeout = rclcpp::Duration::from_seconds(10.0);
        // Send Request
        auto result = switch_controller->async_send_request(srv);
        // Wait for the result (blocking)
        if (rclcpp::spin_until_future_complete(n, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            // Service call succeeded
        }
}

/*
    Prepares the simulated velocity publishers for control.
*/
void JointVelocityController::_prepareSimVelocityPublisher(){
    velocityPublishers.reserve(7);
    for (int i = 0; i < 7; i++) {
        std::string topic_name = "panda_joint" + std::to_string(i+1) + "_velocity_controller/command";
        velocityPublishers.push_back(n->create_publisher<std_msgs::msg::Float64>(topic_name, 10));
    }
}


void JointVelocityController::_prepareRealVelocityPublisher(){
    realVelocityPublisher = n->create_publisher<std_msgs::msg::Float64MultiArray>(
        // "panda_joint_velocity_controller/command", 1);
        "/panda_joint_velocity_controller/command", 1);
}

/*
    Send Joint Velocity Commands
*/
void JointVelocityController::sendVelocities(const Eigen::VectorXd velocities){
    if (velocities.size() != 7) {
        RCLCPP_ERROR_ONCE(n->get_logger(), "The published vector must contain 7 elements");
    }

    if (this->isSim){
        _sendSimVelocities(velocities);
        // _sendRealVelocities(velocities);

    } else {
        _sendRealVelocities(velocities);
    }
}

/*
    Send joint velocities in simulation
*/
void JointVelocityController::_sendSimVelocities(const Eigen::VectorXd velocities){
    for (int i = 0; i < 7; i++){
        // double velocity = velocities[i];
        if (std::isnan(velocities[i]))
        {
            RCLCPP_ERROR(n->get_logger(), "NaN values can't be published as joint velocities");
            RCLCPP_ERROR(n->get_logger(), "Shutting down");
            rclcpp::shutdown();
        }

        msg.data = velocities[i];
        velocityPublishers[i]->publish(msg);
    }
}


/*
    Send joint velocities to the physical robot
*/
void JointVelocityController::_sendRealVelocities(const Eigen::VectorXd velocities){
    for (int i = 0; i < 7; i++){
        // double velocity = velocities[i];
        if (std::isnan(velocities[i]))
        {
            RCLCPP_ERROR(n->get_logger(), "NaN values can't be published as joint velocities");
            RCLCPP_ERROR(n->get_logger(), "Shutting down");
            rclcpp::shutdown();
        }
        msgarray.data[i] = velocities[i];
    }
    realVelocityPublisher->publish(msgarray);
}
