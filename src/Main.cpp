#include "CartesianPositionController.h"
#include <deque>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <deque>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <chrono>

// Forward declare the node pointer
std::shared_ptr<rclcpp::Node> g_node;

/*
    function called on a shutdown of this node.
*/
void on_shutdown(int sig) {
    CartesianPositionController endController(true);
    endController.setVelocitiesToZero();
    rclcpp::shutdown();
}

Eigen::Vector3d goalPosG = Eigen::Vector3d(0.5, 0, 0.35);
Eigen::Vector4d xboxVals = Eigen::Vector4d(0, 0, 0, 0);


void setAvoidanceMode(CartesianPositionController &controller, std::shared_ptr<rclcpp::Node> node){

    std::string control_mode;
    node->declare_parameter("avoidance_type", "hiro");
    node->get_parameter("avoidance_type", control_mode);
    RCLCPP_INFO(node->get_logger(), "Got avoidance_type parameter: %s", control_mode.c_str());
    
    if (control_mode == "hiro") {
        controller.setAvoidanceMode(HIRO);
        RCLCPP_INFO(node->get_logger(), "HIRO avoidance selected");
    } else if (control_mode == "none") {
        controller.setAvoidanceMode(noAvoidance);
        RCLCPP_INFO(node->get_logger(), "No avoidance selected");
    } else {
        RCLCPP_WARN(node->get_logger(), "Unknown avoidance type: %s, defaulting to HIRO", control_mode.c_str());
        controller.setAvoidanceMode(HIRO);
    }
}

void moveSequenceOfPositions(CartesianPositionController &controller, std::string positions_filename, std::shared_ptr<rclcpp::Node> node){

    auto gripper_pub = node->create_publisher<std_msgs::msg::String>("/hiro_sequence", 1);

    while(true){
        std::ifstream in(positions_filename);
        for (std::string f; getline(in, f);)
        {
            std::istringstream iss(f);
            std::vector<double> cartesian_position;
            std::copy(std::istream_iterator<double>(iss),
                std::istream_iterator<double>(),
                std::back_inserter(cartesian_position));
            Eigen::Vector3d desired_point = Eigen::Vector3d(cartesian_position[0],
                                                            cartesian_position[1],
                                                            cartesian_position[2]);
        Eigen::Vector3d endEffectorPosition;
        endEffectorPosition = controller.getEEPosition();

        Eigen::Vector3d positionError = desired_point - endEffectorPosition;
        // If this value is true then pause the movement of the robot for a moment and issue a grab action
        if(cartesian_position[5]){
            //Publish a grasp and sleep for 2 sec
            std_msgs::msg::String msg;
            msg.data = "swap_gripper_state";
            gripper_pub->publish(msg);
            rclcpp::sleep_for(std::chrono::seconds(2));

        }else{
            while (positionError.norm() > 0.005){
                    controller.moveToPositionOneStepSeq(desired_point, cartesian_position[3], cartesian_position[4]);
                    endEffectorPosition = controller.getEEPosition();
                    positionError = desired_point - endEffectorPosition;
                }                                                        
            }
        }
            // loop until position error is sufficiently small
        
    }
    controller.setVelocitiesToZero();
}

void desiredStateCallback(const geometry_msgs::msg::Point::SharedPtr msg){
    RCLCPP_INFO(g_node->get_logger(), "desiredStateCallback");
    std::cout << "x: " << msg->x << "  y: " << msg->y << "  z: " << msg->z << std::endl;
    goalPosG = Eigen::Vector3d(msg->x, msg->y, msg->z);

}

void moveToSubPoints(CartesianPositionController &controller){
    //
    //controller.moveToPosition(goalPosG);
    rclcpp::Rate r(1);

    Eigen::Vector3d trajectory, endEffectorPosition, positionError;
    Eigen::Vector3d goal = Eigen::Vector3d(0.5, 0, 0.6);
    endEffectorPosition = controller.getEEPosition();


    // good version
    float p_gain = 0.45;
    float i_gain = 0.18;
    float d_gain = 0.0;

    Eigen::Vector3d prev_error;
    Eigen::Vector3d cum_error = Eigen::Vector3d(0, 0, 0);
    float max_vel = 0;

    Eigen::Vector3d desiredEEVelocity;

    rclcpp::Time start_of_movement = g_node->now();
    rclcpp::Time cur_time = start_of_movement;

    rclcpp::Rate rate{100.0};

    // increment
    bool done = false;
    float error;
    rclcpp::Duration cur_time_duration = rclcpp::Duration::from_seconds(0);
    double delta_time = 0.01;
    double total_error = 0.0;
    double prev = 0;
    Eigen::Vector3d controller_output;
    Eigen::Vector3d prev_output;
    Eigen::Vector3d acc;

    int i = 0;

    while (true)
    {
        endEffectorPosition = controller.getEEPosition();
        error = abs((goalPosG - endEffectorPosition).norm());
        endEffectorPosition = controller.getEEPosition();

        // desired velocity
        desiredEEVelocity = (goalPosG - endEffectorPosition) / (0.01 * 5); // limit ee velocity
        // proportional error
        Eigen::Vector3d p_error = (goalPosG - endEffectorPosition);
        // integral error
        cum_error += (p_error * 0.01);
        Eigen::Vector3d i_error = (cum_error);
        // derivative error
        Eigen::Vector3d d_error = (p_error - prev_error) / 0.01 ;
        if (i == 0)
        {
            d_error = Eigen::Vector3d(0, 0, 0);
        }


        // controller_output = (p_gain * p_error) + (i_gain * i_error) + (d_error * d_gain);
        controller_output = (p_gain * p_error);
        double norm = controller_output.norm();
        acc = (controller_output - prev_output) / 0.01;

        controller.commandVelocityOneStep(controller_output, goalPosG);

        cur_time_duration = (g_node->now() - start_of_movement);
        delta_time = cur_time_duration.seconds() - prev;
        prev = cur_time_duration.seconds();
        i++;
        prev_error = p_error;
        prev_output = controller_output;
        // r.sleep();

    }
}


/*
    Move the robot in a circle
*/
void moveInCircle(CartesianPositionController &controller, double timeToComplete){
    double radius = 0.1;
    Eigen::Vector3d start_pos = Eigen::Vector3d(0.5, radius, 0.35);
    controller.moveToPosition(start_pos);

    Eigen::Vector3d trajectory;
    double theta = 0;
    // equation for a circle -- x doesn't change
    double x = 0.5;
    double y = radius * std::cos(theta);
    // double z = 0.5 + radius * std::sin(theta);
    Eigen::Vector3d center = Eigen::Vector3d(0.5, 0, start_pos(1));
    double z = start_pos(2) + radius * std::sin(theta);
    bool circle_complete = false;
    // double time_diff;
    // double end_time;
    // rclcpp::Time start_of_movement = g_node->now();


    rclcpp::Rate rate{100.0};
    trajectory = Eigen::Vector3d(x, y, z);
    // increment
    double inc = (M_PI * 2.0) / (timeToComplete * 100.0);
    bool done = false;
    float error;


    Eigen::Vector3d goal, endEffectorPosition, positionError;
    while (!circle_complete)
    {
        goal = trajectory;
        error = abs((trajectory - endEffectorPosition).norm());
        controller.moveToPositionOneStep(trajectory);
        theta = theta + inc;
        y = radius * std::cos(theta);
        z = start_pos(2) + radius * std::sin(theta);
        trajectory = Eigen::Vector3d(x, y, z);


        endEffectorPosition = controller.getEEPosition();
        positionError = center - endEffectorPosition;

        float error = abs(positionError.norm() - radius);
        // Note: Logging removed - was: logPublisher.publishEE_Error(error);
        // Note: Logging removed - was: logPublisher.publishEEPath(trajectory);
        // Note: Logging removed - was: logPublisher.publishEETrajectory(endEffectorPosition, theta);
        // rate.sleep();
    }
    controller.setVelocitiesToZero();
}

void holdStaticPose(CartesianPositionController &controller){

    Eigen::Vector3d trajectory, endEffectorPosition, positionError;
    Eigen::Vector3d goal = Eigen::Vector3d(0.4, -0.2, 0.45);
    endEffectorPosition = controller.getEEPosition();


    // good version
    float p_gain = 0.45;
    float i_gain = 0.18;
    float d_gain = 0.0;

    Eigen::Vector3d prev_error;
    Eigen::Vector3d cum_error = Eigen::Vector3d(0, 0, 0);
    float max_vel = 0;

    Eigen::Vector3d desiredEEVelocity;

    rclcpp::Time start_of_movement = g_node->now();
    rclcpp::Time cur_time = start_of_movement;

    rclcpp::Rate rate{100.0};

    // increment
    bool done = false;
    float error;
    rclcpp::Duration cur_time_duration = rclcpp::Duration::from_seconds(0);
    double delta_time = 0.01;
    double total_error = 0.0;
    double prev = 0;
    Eigen::Vector3d controller_output;
    Eigen::Vector3d prev_output;
    Eigen::Vector3d acc;

    int i = 0;

    while (true)
    {
        // RCLCPP_INFO(g_node->get_logger(), "Holding static pose, Moving to position: %f, %f, %f", goal(0), goal(1), goal(2));
        endEffectorPosition = controller.getEEPosition();
        error = abs((goal - endEffectorPosition).norm());
        endEffectorPosition = controller.getEEPosition();

        // desired velocity
        desiredEEVelocity = (goal - endEffectorPosition) / (0.01 * 5); // limit ee velocity
        // proportional error
        Eigen::Vector3d p_error = (goal - endEffectorPosition);
        // integral error
        cum_error += (p_error * 0.01);
        Eigen::Vector3d i_error = (cum_error);
        // derivative error
        Eigen::Vector3d d_error = (p_error - prev_error) / 0.01 ;
        if (i == 0)
        {
            d_error = Eigen::Vector3d(0, 0, 0);
        }


        // controller_output = (p_gain * p_error) + (i_gain * i_error) + (d_error * d_gain);
        controller_output = (p_gain * p_error);
        double norm = controller_output.norm();
        acc = (controller_output - prev_output) / 0.01;


        controller.publishGoal(goal);
        controller.moveToPositionOneStep(goal);

        cur_time_duration = (g_node->now() - start_of_movement);
        delta_time = cur_time_duration.seconds() - prev;
        prev = cur_time_duration.seconds();
        i++;
        prev_error = p_error;
        prev_output = controller_output;

    }
    controller.setVelocitiesToZero();
}


/*
    also moves the robot in a circle -- but eliminates the controller error.
*/
void moveInCircleV2(CartesianPositionController &controller, double timeToComplete){
    double radius = 0.25;
    Eigen::Vector3d goal, endEffectorPosition, positionError;
    Eigen::Vector3d start_pos = Eigen::Vector3d(0.5, radius, 0.35);
    controller.moveToPosition(start_pos);
    endEffectorPosition = controller.getEEPosition();

    // oscillates at 17
    // float p_gain = 0.45;
    // float i_gain = 0.18;
    // float d_gain = 0.0;
    // queue that [can] be used
    std::deque<Eigen::Vector3d> basic_queue;
    // good version
    float p_gain = 3.00;
    float i_gain = 0.00;
    float d_gain = 0.0;
    // this gain's error is higher -- but may be more unstable in real life~
    // float p_gain = 10;
    // float i_gain = 100.0;
    // float d_gain = 0.05;

    Eigen::Vector3d prev_error;
    Eigen::Vector3d cum_error = Eigen::Vector3d(0, 0, 0);
    float max_vel = 0;

    Eigen::Vector3d trajectory;
    Eigen::Vector3d desiredEEVelocity;

    double theta = 0;

    // equation for a circle
    float y1, z1;
    double x = 0.5;
    double y = radius * std::cos(theta);
    // double z = 0.5 + radius * std::sin(theta);
    Eigen::Vector3d center = Eigen::Vector3d(0.5, 0, start_pos(1));
    double z = start_pos(2) + radius * std::sin(theta);
    bool circle_complete = false;

    rclcpp::Time start_of_movement = g_node->now();
    rclcpp::Time cur_time = start_of_movement;

    rclcpp::Rate rate{100.0};
    trajectory = Eigen::Vector3d(x, y, z);

    // increment
    double b = (M_PI * 2.0) / timeToComplete;
    double inc = (M_PI * 2.0) / (timeToComplete * 100.0);
    bool done = false;
    float error;
    rclcpp::Duration cur_time_duration = rclcpp::Duration::from_seconds(0);
    double delta_time = 0.01;
    double total_error = 0.0;
    double prev = 0;
    float prev_norm = 0;
    Eigen::Vector3d controller_output;
    Eigen::Vector3d prev_output;
    Eigen::Vector3d acc;
    Eigen::Vector3d prev_trajectory;


    endEffectorPosition = Eigen::Vector3d(0.5, radius, 0.35);
    // a potentially useful incremental var
    int i = 0;
    float amt = 0;
    while (!circle_complete)
    {
        endEffectorPosition = controller.getEEPosition();
        error = abs((trajectory - endEffectorPosition).norm());
        total_error = total_error + error;
        float t = (rclcpp::Time(start_of_movement).seconds() + amt);
        y = radius * std::cos(b * t);
        z = start_pos(2) + radius * std::sin(b * t);
        prev_trajectory = trajectory;
        trajectory = Eigen::Vector3d(x, y, z);
        Eigen::Vector3d des_vel = (trajectory - prev_trajectory) / 0.01;
        Eigen::Vector3d d_error = des_vel - controller.getEEVelocity();
        endEffectorPosition = controller.getEEPosition();


        // desired velocity
        desiredEEVelocity = (trajectory - endEffectorPosition) / (0.01 * 5); // limit ee velocity
        // proportional error
        Eigen::Vector3d p_error = (trajectory - endEffectorPosition);
        // integral error
        cum_error += (p_error * 0.01);
        Eigen::Vector3d i_error = (cum_error);
        // derivative error
        if (i == 0)
        {
            d_error = Eigen::Vector3d(0, 0, 0);
        }
        controller_output = (p_gain * p_error) + (i_gain * i_error) + (des_vel * d_gain);
        double norm = controller_output.norm();
        double diff = norm - prev_norm;
        double scale = std::max(diff, -0.001);
        scale = std::min(scale, 0.001);
        acc = (controller_output - prev_output) / 0.01;
        for (int i = 0; i < 3; i++)
        {
            float total_sum = controller_output(i);
            for (int j = 0; j < basic_queue.size(); j++)
            {
                total_sum += basic_queue[j](i);
            }
            float avg = total_sum / (basic_queue.size() + 1);
            // if the below line is *uncommented", then the output
            // will go through a moving avg filter
            // controller_output(i) = avg;
        }

        controller.commandVelocityOneStep(controller_output, trajectory);
        cur_time_duration = (g_node->now() - start_of_movement);
        delta_time = cur_time_duration.seconds() - prev;
        prev = cur_time_duration.seconds();
        i++;
        amt = amt + 0.01;
        prev_error = p_error;
        prev_output = controller_output;
        prev_norm = norm;
        if (basic_queue.size() + 1 > 4)
        {
            basic_queue.pop_front();

        }
        basic_queue.push_back(controller_output);

    }
    // end task and set joint velocities to zero
    controller.setVelocitiesToZero();
}




/*
    moves the robot in a line, back and forth
*/
void moveInLine(CartesianPositionController &controller, double time_to_complete){
    Eigen::Vector3d start_pos = Eigen::Vector3d(0.1, 0.0, 0.65);
    Eigen::Vector3d goal_pos = Eigen::Vector3d(0.85, 0.0, 0.35);

    controller.moveToPosition(start_pos);
    bool negative_direction = false;
    bool done = false;
    double end_time;
    Eigen::Vector3d trajectory;
    rclcpp::Time now;
    rclcpp::Time start = g_node->now();
    rclcpp::Time start_of_movement = start;
    Eigen::Vector3d goal;
    Eigen::Vector3d endEffectorPosition, positionError;

    goal = goal_pos;
    double error_threshold = 0.05; // Determines when we have reached the goal in meters

    while (true)
    {
        endEffectorPosition = controller.getEEPosition();
        positionError = goal - endEffectorPosition;

        // Update time
        now = g_node->now();
        end_time = (now - start_of_movement).seconds();

        //Check if over time
        if (end_time >= time_to_complete) done = true;

        //Move the robot towards the goal
        // controller.moveToPositionOneStep(goal);
        controller.moveToPosition(goal);
        //Check if the direction needs to be switched
        if (negative_direction){
            if (positionError.squaredNorm() < error_threshold) {
                goal = start_pos;
                negative_direction = false;
                start = g_node->now();
            }
        }
        else
        {
            if (positionError.squaredNorm() < error_threshold) {
                goal = goal_pos;
                negative_direction = true;
                start = g_node->now();
            }
        }
    }
    controller.setVelocitiesToZero();
}

bool getSimParam(std::shared_ptr<rclcpp::Node> node){

    bool is_sim;
    node->declare_parameter("is_sim", false);
    node->get_parameter("is_sim", is_sim);
    if (is_sim){
        RCLCPP_INFO(node->get_logger(), "Simulation: True");
    } else {
        RCLCPP_INFO(node->get_logger(), "Simulation: False");
    }
    return is_sim;
}

void moveRobot(CartesianPositionController &controller, std::shared_ptr<rclcpp::Node> node){

    std::string movement_mode;
    node->declare_parameter("movement_type", "");
    node->get_parameter("movement_type", movement_mode);
    if (movement_mode == "line"){
        RCLCPP_INFO(node->get_logger(), "Move in line");
        moveInLine(controller, 10);
    } else if (movement_mode == "circle"){
        RCLCPP_INFO(node->get_logger(), "Move in circle");
        moveInCircle(controller, 5);
    } else if (movement_mode == "circlev2"){
        RCLCPP_INFO(node->get_logger(), "Move in circle v2");
        moveInCircleV2(controller, 10);
    } else if(movement_mode == "static"){
        RCLCPP_INFO(node->get_logger(), "static position");
        holdStaticPose(controller);
    } else if(movement_mode == "send_xyz"){
        RCLCPP_INFO(node->get_logger(), "Send xyz geometry point message to /panda_desired_pos_xyz");
        moveToSubPoints(controller);
    }else if(movement_mode == "sequence"){
        RCLCPP_INFO(node->get_logger(), "sequence mode set");
        moveSequenceOfPositions(controller, "/home/panda/panda_positions.txt", node);
    }else{
        RCLCPP_ERROR(node->get_logger(), "No movement mode selected");
        rclcpp::shutdown();
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("CartesianPositionController");
    signal(SIGINT, on_shutdown);
    // auto subscriberCartPos_xyz = g_node->create_subscription<geometry_msgs::msg::Point>(
    //     "/panda_desired_pos_xyz", 1, &desiredStateCallback);
    // auto subscriberJoyXbox = g_node->create_subscription<sensor_msgs::msg::Joy>(
    //     "/joy", 1, &xboxStateCallback);
    
    RCLCPP_INFO(g_node->get_logger(), "Main node initialized");

    bool isSim = getSimParam(g_node);

    RCLCPP_INFO(g_node->get_logger(), "Simulation: %d", isSim);


    CartesianPositionController controller(isSim, g_node);

    RCLCPP_INFO(g_node->get_logger(), "Controller initialized");

    //TODO: Make the avoidance mode a controller function
    setAvoidanceMode(controller, g_node);

    RCLCPP_INFO(g_node->get_logger(), "Avoidance mode set");

    //This is dependent on the argument set in the launch file
    moveRobot(controller, g_node);


    return 0;
}
