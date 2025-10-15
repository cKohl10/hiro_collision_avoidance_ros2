#include "CartesianPositionController.h"
#include <deque>
#include <sensor_msgs/Joy.h>

#include <fstream>
#include <deque>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

/*
    function called on a shutdown of this node.
*/
void on_shutdown(int sig) {
    CartesianPositionController endController(true);
    endController.setVelocitiesToZero();
    ros::shutdown();
}

Eigen::Vector3d goalPosG = Eigen::Vector3d(0.5, 0, 0.35);
Eigen::Vector4d xboxVals = Eigen::Vector4d(0, 0, 0, 0);


void setAvoidanceMode(CartesianPositionController &controller, ros::NodeHandle nh){

    std::string control_mode;
    nh.getParam("/avoidance_controller/avoidance_type", control_mode);
    ROS_INFO("Got parameter : %s", control_mode.c_str());
    if (control_mode == "" || control_mode == "none"){
        ROS_INFO("No avoidance selected");
        controller.setAvoidanceMode(noAvoidance);
    } else if (control_mode == "flacco"){
        ROS_INFO("flacco avoidance selected");
        controller.setAvoidanceMode(Flacco);
    } else if (control_mode == "ding"){
        ROS_INFO("ding avoidance selected");
        controller.setAvoidanceMode(Ding);
    } else if (control_mode == "hiro"){
        ROS_INFO("hiro avoidance selected");
        controller.setAvoidanceMode(HIRO);
    } else if (control_mode == "HIROCollaborative"){
        ROS_INFO("HIRO *collaborative* controller selected");
        controller.setAvoidanceMode(HIROCollaborative);
    } else{
        ROS_ERROR("Mode entered is not valid");
        ros::shutdown();
    }

}
void moveSequenceOfPositions(CartesianPositionController &controller, std::string positions_filename, ros::NodeHandle &nh){

    ros::Publisher gripper_pub = nh.advertise<std_msgs::String>("/hiro_sequence",1);

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
            std_msgs::String msg;
            msg.data = "swap_gripper_state";
            gripper_pub.publish(msg);
            ros::Duration(2.0).sleep();

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

void xboxStateCallback(const sensor_msgs::Joy::ConstPtr& joy){
    ROS_INFO("Xbox Callback!");
    std::cout << "x: " << joy->axes[0] << "  y: " << joy->axes[1] << "  z: " << joy->axes[4] << std::endl;
    xboxVals = Eigen::Vector4d(joy->axes[0], joy->axes[1], joy->axes[4], 0);
}

void desiredStateCallback(const geometry_msgs::Point& msg){
    ROS_INFO("desiredStateCallback");
    std::cout << "x: " << msg.x << "  y: " << msg.y << "  z: " << msg.z << std::endl;
    goalPosG = Eigen::Vector3d(msg.x, msg.y, msg.z);

}
void xboxController(CartesianPositionController &controller){

    //controller.moveToPosition(goalPosG);
    ros::Rate r(1);

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

    ros::Time start_of_movement = ros::Time::now();
    ros::Time cur_time = start_of_movement;

    ros::Rate rate{100.0};

    // increment
    bool done = false;
    float error;
    ros::Duration cur_time_duration = ros::Duration(0);
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
        goalPosG = Eigen::Vector3d(endEffectorPosition(0) + xboxVals(1) * 0.3 ,  endEffectorPosition(1) + xboxVals(0) * 0.3, endEffectorPosition(2) + xboxVals(2) * 0.3);
        std::cout << goalPosG(0) << std::endl;
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
        // controller.moveToPositionOneStep(goalPosG);

        cur_time_duration = (ros::Time::now() - start_of_movement);
        delta_time = cur_time_duration.toSec() - prev;
        prev = cur_time_duration.toSec();
        i++;
        prev_error = p_error;
        prev_output = controller_output;
        // r.sleep();

    }
}

void moveToSubPoints(CartesianPositionController &controller){
    //
    //controller.moveToPosition(goalPosG);
    ros::Rate r(1);

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

    ros::Time start_of_movement = ros::Time::now();
    ros::Time cur_time = start_of_movement;

    ros::Rate rate{100.0};

    // increment
    bool done = false;
    float error;
    ros::Duration cur_time_duration = ros::Duration(0);
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

        cur_time_duration = (ros::Time::now() - start_of_movement);
        delta_time = cur_time_duration.toSec() - prev;
        prev = cur_time_duration.toSec();
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
    LoggingPublisher logPublisher;
    double radius = 0.4;
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
    // ros::Time start_of_movement = ros::Time::now();


    ros::Rate rate{100.0};
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
        // utilize logging publisher - publish ee pos
        logPublisher.publishEE_Error(error);
        // logPublisher.publishEEPath(endEffectorPosition);
        logPublisher.publishEEPath(trajectory);
        logPublisher.publishEETrajectory(endEffectorPosition, theta);
        // rate.sleep();
    }
    controller.setVelocitiesToZero();
}

void holdStaticPose(CartesianPositionController &controller){

    Eigen::Vector3d trajectory, endEffectorPosition, positionError;
    Eigen::Vector3d goal = Eigen::Vector3d(0.4, 0, 0.45);
    endEffectorPosition = controller.getEEPosition();


    // good version
    float p_gain = 0.45;
    float i_gain = 0.18;
    float d_gain = 0.0;

    Eigen::Vector3d prev_error;
    Eigen::Vector3d cum_error = Eigen::Vector3d(0, 0, 0);
    float max_vel = 0;

    Eigen::Vector3d desiredEEVelocity;

    ros::Time start_of_movement = ros::Time::now();
    ros::Time cur_time = start_of_movement;

    ros::Rate rate{100.0};

    // increment
    bool done = false;
    float error;
    ros::Duration cur_time_duration = ros::Duration(0);
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


        controller.moveToPositionOneStep(goal);

        cur_time_duration = (ros::Time::now() - start_of_movement);
        delta_time = cur_time_duration.toSec() - prev;
        prev = cur_time_duration.toSec();
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

    ros::Time start_of_movement = ros::Time::now();
    ros::Time cur_time = start_of_movement;

    ros::Rate rate{100.0};
    trajectory = Eigen::Vector3d(x, y, z);

    // increment
    double b = (M_PI * 2.0) / timeToComplete;
    double inc = (M_PI * 2.0) / (timeToComplete * 100.0);
    bool done = false;
    float error;
    ros::Duration cur_time_duration = ros::Duration(0);
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
        float t = (start_of_movement.toSec() + amt);
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
        cur_time_duration = (ros::Time::now() - start_of_movement);
        delta_time = cur_time_duration.toSec() - prev;
        prev = cur_time_duration.toSec();
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
    ros::Time now;
    ros::Time start = ros::Time::now();
    ros::Time start_of_movement = start;
    Eigen::Vector3d goal;
    Eigen::Vector3d endEffectorPosition, positionError;

    goal = goal_pos;
    double error_threshold = 0.05; // Determines when we have reached the goal in meters

    while (true)
    {
        endEffectorPosition = controller.getEEPosition();
        positionError = goal - endEffectorPosition;

        // Update time
        now = ros::Time::now();
        end_time = (now - start_of_movement).toSec();

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
                start = ros::Time::now();
            }
        }
        else
        {
            if (positionError.squaredNorm() < error_threshold) {
                goal = goal_pos;
                negative_direction = true;
                start = ros::Time::now();
            }
        }
    }
    controller.setVelocitiesToZero();
}

bool getSimParam(ros::NodeHandle &nh){

    bool is_sim;
    nh.getParam("/is_sim", is_sim);
    if (is_sim){
        ROS_INFO("Simulation: True");
        return true;
    } else if (!is_sim){
        ROS_INFO("Simulation: False");
        return false;
    } else{
        ROS_ERROR("is_sim is not set, please set this variable to: true or false");
        ros::shutdown();
    }
}

void moveRobot(CartesianPositionController &controller, ros::NodeHandle nh){

    std::string movement_mode;
    nh.getParam("/avoidance_controller/movement_type", movement_mode);
    if (movement_mode == "line"){
        ROS_INFO("Move in line");
        moveInLine(controller, 10);
    } else if (movement_mode == "circle"){
        ROS_INFO("Move in circle");
        moveInCircle(controller, 5);
    } else if (movement_mode == "circlev2"){
        ROS_INFO("Move in circle v2");
        moveInCircleV2(controller, 10);
    } else if(movement_mode == "static"){
        ROS_INFO("static position");
        holdStaticPose(controller);
    } else if(movement_mode == "send_xyz"){
        ROS_INFO("Send xyz geometry point message to /panda_desired_pos_xyz");
        moveToSubPoints(controller);
    }else if(movement_mode == "xbox"){
        ROS_INFO("xbox controller mode set");
        xboxController(controller);
    }else if(movement_mode == "sequence"){
        ROS_INFO("sequence mode set");
        moveSequenceOfPositions(controller, "/home/panda/panda_positions.txt", nh);
    }else{
        ROS_ERROR("No movement mode selected");
        ros::shutdown();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "CartesianPositionController", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    signal(SIGINT, on_shutdown);
    ros::Subscriber subscriberCartPos_xyz = nh.subscribe("/panda_desired_pos_xyz", 1, &desiredStateCallback);
    ros::Subscriber subscriberJoyXbox = nh.subscribe("/joy", 1, &xboxStateCallback);
    

    bool isSim = getSimParam(nh);


    CartesianPositionController controller(isSim, nh);

    //TODO: Make the avoidance mode a controller function
    setAvoidanceMode(controller, nh);

    //This is dependent on the argument set in the launch file
    moveRobot(controller, nh);

    return 0;
}
