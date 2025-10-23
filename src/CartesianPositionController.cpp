#include "CartesianPositionController.h"
#include <fstream>
#include <chrono>
#include <functional>

/*
    set the joint limits for the robot,
    position, velocity, and acceleration limits
*/
void CartesianPositionController::setJointLimits(std::shared_ptr<rclcpp::Node> node_handle){
    Eigen::VectorXd jointLimitsMin{7}, jointLimitsMax{7}, jointMiddleValues{7}, jointRanges{7}, jointVelocityMax{7}, jointAccelerationMax{7};

    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointVelocityMax << 2.1750, 2.1750, 2.1750 , 2.1750, 2.6100 , 2.6100 , 2.6100;
    jointAccelerationMax << 15, 7.5, 10, 12.5, 15, 20, 20;


    //TODO: remove hard limits on joint vel and acc
    // jointAccelerationMax = jointAccelerationMax * 0.001;
    jointAccelerationMax = jointAccelerationMax * 0.3;
    // TODO(peasant98) -- why is this needed?
    jointVelocityMax = jointVelocityMax * 0.5;

    jointMiddleValues = 0.5 * (jointLimitsMax + jointLimitsMin);
    jointRanges = jointLimitsMax - jointLimitsMin;

    this->jointLimits = JointLimits(jointLimitsMin, jointLimitsMax, jointMiddleValues, jointRanges, jointVelocityMax, jointAccelerationMax);
    hiroAvoidance = HIROAvoidance(this->jointLimits, node_handle);
    qpAvoidance = QPAvoidance(this->jointLimits);

    q.resize(7);
}

/*
    get the control points.
*/
void CartesianPositionController::getControlPoints(){
    //TODO: Are these static or dynamic at this point?
    this->numberControlPoints = kdlSolver.getNumberControlPoints();
    this->controlPointPositionVectors = std::make_unique<Eigen::Vector3d[]>(numberControlPoints);
}

/*
    create the ROS publishers and subscribers:
    joint states subscriber
    obstacle points subscriber
    arrow publisher
*/
void CartesianPositionController::createRosPubsSubs(){
    subscriberJointStates = n->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1,
                                         std::bind(&CartesianPositionController::JointStateCallback, this, std::placeholders::_1));
    subscriberObstaclePoints = n->create_subscription<sensor_msgs::msg::PointCloud2>("/obstacle_pointcloud", 10,
                                                      std::bind(&CartesianPositionController::ObstaclePointsCallback, this, std::placeholders::_1));
    
    // TODO: Naming convention for fr3
    subscriberExternalCartesianWrench = n->create_subscription<std_msgs::msg::Float64MultiArray>("/panda_joint_velocity_contact_controller/external_signal", 1,
                                                      std::bind(&CartesianPositionController::ExternalCartesianWrenchCallback, this, std::placeholders::_1));
    subscriberExternalCartesianWrenchRaw = n->create_subscription<std_msgs::msg::Float64MultiArray>("/panda_joint_velocity_contact_controller/external_wrench", 1,
                                                      std::bind(&CartesianPositionController::ExternalCartesianWrenchRawCallback, this, std::placeholders::_1));
    subscriberControllerSwitch = n->create_subscription<std_msgs::msg::String>("/controller_switch", 1,
                                                       std::bind(&CartesianPositionController::ControllerSwitchCallback, this, std::placeholders::_1));


    //TODO: Remove these publishers
    // All visualization is done inside of each controller class 
    // Throught he LoggerPublisher
    // arrow_publisher =  n->create_publisher<std_msgs::msg::Float64>("/contact_threshold",1);
    // test_pub =  n->create_publisher<std_msgs::msg::Float64>("/current_vel_des",1);

}

/*
    find the closest control points to obstacles
*/
void CartesianPositionController::findClosestControlPointsToObstacles(){

    // use forward kinematics
    joint_positions = kdlSolver.forwardKinematicsJoints(q);

    // Initalize a list to hold the starting segemnt of the closest line
    closestPointsOnRobot.clear();
    closestPointsOnRobot.resize(obstaclePositionVectors.size());

    // Temporary variables
    float cur_dist;
    double cur_t;
    Eigen::Vector3d cur_control_point;

    // Loop though all obstacle points to find the closest position on the robot to each
    Eigen::Vector3d starting_point, ending_point;
    std::vector<int> joints = {2, 3, 4, 6, 7, 9};
    for (int obs = 0; obs < obstaclePositionVectors.size(); obs++)
    {
        for (int i = 0; i < joint_positions.cols() - 1; i++)
        {
            starting_point = joint_positions.col(i);
            ending_point = joint_positions.col(i+1);
            cur_control_point = getClosestPointOnLine(starting_point, ending_point, obstaclePositionVectors[obs], cur_t);
            cur_dist = (obstaclePositionVectors[obs] - cur_control_point).norm();

            if (cur_dist < closestPointsOnRobot[obs].distance_to_obs)
            {
                // control Point on robot closer to obstacle than the previous
                // closest control point

                closestPointsOnRobot[obs].segmentId = joints[i];
                closestPointsOnRobot[obs].segmentPointA = starting_point;
                closestPointsOnRobot[obs].segmentPointB = ending_point;
                if (cur_dist > 0.01){
                    closestPointsOnRobot[obs].distance_to_obs = cur_dist;
                }
                closestPointsOnRobot[obs].control_point = cur_control_point;
                closestPointsOnRobot[obs].t = cur_t;
            }
        }
    }
}

/*
    callback for detected wrench
*/
void CartesianPositionController::ExternalCartesianWrenchCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // set current ext cartesian generalized force from the callback
    externalCartesianForce << msg->data[0],
                              msg->data[1],
                              msg->data[2];
}

void CartesianPositionController::ControllerSwitchCallback(const std_msgs::msg::String::SharedPtr msg){
    const std::lock_guard<std::mutex> lock(mutex);
    std::string controllerName = msg->data;
    if(controllerName == "hiro"){
        setAvoidanceMode(HIRO);
        RCLCPP_INFO(n->get_logger(), "Controller -> hiro");
    }else if(controllerName == "none"){
        setAvoidanceMode(noAvoidance);
        RCLCPP_INFO(n->get_logger(), "Controller -> none");
    }else{
        std::string s = "No controller with name " + controllerName + ". Options: hiro, none ";
        RCLCPP_INFO(n->get_logger(), s.c_str());
    }
}

void CartesianPositionController::ExternalCartesianWrenchRawCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // set current ext cartesian generalized force from the callback
    externalCartesianWrench << msg->data[0],
                               msg->data[1],
                               msg->data[2];
}


/*
    callback for joint states.
*/

void CartesianPositionController::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    if (this->isSim) {
        // indices 2..8 will be accessed; require at least 9 elements
        if (msg->position.size() < 9 || msg->velocity.size() < 9) {
            RCLCPP_WARN_THROTTLE(n->get_logger(), *n->get_clock(), 2000,
                                 "JointState too small for sim indexing: pos=%zu vel=%zu",
                                 msg->position.size(), msg->velocity.size());
            return;
        }
        q << msg->position[2], msg->position[3], msg->position[4],
             msg->position[5], msg->position[6], msg->position[7],
             msg->position[8];

        current_Qdot << msg->velocity[2], msg->velocity[3], msg->velocity[4],
             msg->velocity[5], msg->velocity[6], msg->velocity[7],
             msg->velocity[8];

    } else {
        // indices 0..6 will be accessed; require at least 7 elements
        if (msg->position.size() < 7 || msg->velocity.size() < 7) {
            RCLCPP_WARN_THROTTLE(n->get_logger(), *n->get_clock(), 2000,
                                 "JointState too small for real indexing: pos=%zu vel=%zu",
                                 msg->position.size(), msg->velocity.size());
            return;
        }
        q << msg->position[2-2], msg->position[3-2], msg->position[4-2],
             msg->position[5-2], msg->position[6-2], msg->position[7-2],
             msg->position[8-2];

        current_Qdot << msg->velocity[2-2], msg->velocity[3-2], msg->velocity[4-2],
             msg->velocity[5-2], msg->velocity[6-2], msg->velocity[7-2],
             msg->velocity[8-2];
    }
}

/*
    get the end effector *position* (x,y, z) -- not pose
*/
void CartesianPositionController::readEndEffectorPosition(){
    while (rclcpp::ok()){
        try{
            // get transform betwen world frame and end effector frame
            auto transform = tf_buffer->lookupTransform(base_link_name_, ee_link_name_, tf2::TimePointZero);
            endEffectorPosition << transform.transform.translation.x,
                                         transform.transform.translation.y,
                                         transform.transform.translation.z;
            break;
        }
        catch (tf2::TransformException &ex){
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_ERROR(n->get_logger(), "Failed to lookup transform between world and end effector frame: %s", ex.what());
        }
    }

}

/*
    get the positions of the control points.
*/
void CartesianPositionController::readControlPointPositions(){
    for (int i = 0; i < numberControlPoints; i++){
        // compute positions of each control point with fwd kinematics
        controlPointPositionVectors[i] = kdlSolver.forwardKinematicsControlPoints(std::string("control_point") + std::to_string(i), q);
    }
}

/*
    callback for obstacle points from PointCloud2.
*/
void CartesianPositionController::ObstaclePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    obstaclePositionVectors.clear();
    
    // Use PointCloud2Iterator to efficiently parse the point cloud data
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    // Iterate through all points in the cloud
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        obstaclePositionVectors.push_back(Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
    }
    
    // find closest control points to obstacles
    findClosestControlPointsToObstacles();
}


/*
    convert end effector velocity to joint velocities.
*/
Eigen::VectorXd CartesianPositionController::EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity){

    // compute the jacobian
    // TODO: Naming convention for fr3
    J = kdlSolver.computeJacobian(kdlSolver.getEELink(), q);
    // reshape jacobian
    // TODO: Why do we reshape the J in this way?
    J = J.block(0,0,3,7);
    // get the pseudoinverse of the jacobian, to get the desired joint velocities
    Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    // Move towards the robot's middle joint limits using the null space of the redundant manipulator
    // TODO: Why do we have 2/7 here?
    Eigen::VectorXd secondaryTaskFunctionGradient = 2.0/7.0 * (q - jointLimits.jointMiddleValues).cwiseQuotient(jointLimits.jointRanges);
    return Jpinv * desiredEEVelocity - secondaryTaskGain *
           ((Eigen::MatrixXd::Identity(7, 7) - Jpinv*J)  * secondaryTaskFunctionGradient);
}

Eigen::VectorXd CartesianPositionController::EEVelocityToQDotRepulsive(Eigen::Vector3d desiredEEVelocity, std::vector<Eigen::Vector3d> obstaclePositionVectors){

    readEndEffectorPosition();
    // std::vector<Eigen::Vector3d> fake;

    // Eigen::Vector3d ee_o_error;
    // Eigen::Vector3d rev;
    // Eigen::Vector3d end_pos;
    // double ee_o_mag;
    // double rev_mag;
    // // Go over the obs list and see if we need to move away from the goal
    // for (int i = 0; i < obstaclePositionVectors.size(); i++){
    //     ee_o_error = obstaclePositionVectors[i] - closestPointsOnRobot[i].control_point;
    //     ee_o_mag = ee_o_error.norm();
    //     if ((obstaclePositionVectors[i] - closestPointsOnRobot[i].control_point).norm() < 0.3){
    //         rev = -ee_o_error;
    //         rev_mag = rev.norm();
    //         rev = rev/rev_mag; // This gives us a unit vector for the movement
    //         rev = (1.0 / ee_o_mag) * rev * 0.1;
    //         // rev = (1.0 / ee_o_mag) * rev * 0.22;
    //         // std::cout << "ALTERED" << std::endl;
    //         // desiredEEVelocity = desiredEEVelocity + rev;
    //         desiredEEVelocity = Eigen::Vector3d(0, rev(1), 0);
    //     }
    // }

    // // 12/6/2021 added to allow pushing during repultive controller
    // return hiroAvoidance.computeJointVelocitiesWithExtCartForce(q, desiredEEVelocity,
    //                                                     externalCartesianForce,
    //                                                     externalCartesianWrench,
    //                                                     fake,
    //                                                     closestPointsOnRobot, rate, endEffectorPosition, Eigen::Vector3d(), arrow_publisher, EEVelocity,
    //                                                     robotInContact);

    // compute the jacobian
    // TODO: Naming convention for fr3
    J = kdlSolver.computeJacobian(kdlSolver.getEELink(), q);
    J = J.block(0,0,3,7);
    // get the pseudoinverse of the jacobian in order to get
    Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    // Move towards the robots middle joint limits using the null-space of the redundant manipulator
    Eigen::VectorXd secondaryTaskFunctionGradient =  2.0/7.0 * (q - jointLimits.jointMiddleValues).cwiseQuotient(jointLimits.jointRanges);
    return Jpinv * desiredEEVelocity - secondaryTaskGain *
           ((Eigen::MatrixXd::Identity(7, 7) - Jpinv*J)  * secondaryTaskFunctionGradient);
}

Eigen::Vector3d CartesianPositionController::getEEVelocity(){
    J = kdlSolver.computeJacobian(kdlSolver.getEELink(), q).block(0, 0, 3, 7);
    return J * current_Qdot;
}


/*
    get closest point on line
*/
Eigen::Vector3d CartesianPositionController::getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t)
{
    // TODO(caleb/matt): change variable names
    //https://math.stackexchange.com/a/2193733/801563
    Eigen::Vector3d v = b - a;
    Eigen::Vector3d u = a - p;

    double top = (v.transpose() * u);
    double bottom = (v.transpose() * v);

    t = -top/bottom;

    double d_a = (p - a).norm();
    double d_b = (p - b).norm();

    Eigen::Vector3d c;

    if (0 < t && t < 1){
        c = a + t * (b - a);
    } else {
        if (d_a < d_b){
            c = a;
            t = 0;
        } else {
            c = b;
            t = 1;
        }
    }
    return c;
}

// Public Functions
CartesianPositionController::CartesianPositionController(
    bool isSim, std::shared_ptr<rclcpp::Node> node_handle) {
  this->n = node_handle;
  this->isSim = isSim;
  RCLCPP_INFO(n->get_logger(), "JointVelocityController initializing");
  this->jointVelocityController = JointVelocityController(isSim);

  // Read identifiers from parameters first (no hard-coded defaults)
  std::string arm_id_param;
  std::string ee_link_param;
  std::string base_link_param;
  if (!n->has_parameter("arm_id")) {
    n->declare_parameter<std::string>("arm_id", "");
  }
  if (!n->has_parameter("ee_link")) {
    n->declare_parameter<std::string>("ee_link", "");
  }
  if (!n->has_parameter("base_link")) {
    n->declare_parameter<std::string>("base_link", "");
  }
  n->get_parameter("arm_id", arm_id_param);
  n->get_parameter("ee_link", ee_link_param);
  n->get_parameter("base_link", base_link_param);
  if (arm_id_param.empty() || ee_link_param.empty() || base_link_param.empty()) {
    RCLCPP_ERROR(n->get_logger(), "Missing required parameters: arm_id='%s' ee_link='%s' base_link='%s'",
                 arm_id_param.c_str(), ee_link_param.c_str(), base_link_param.c_str());
  }

  RCLCPP_INFO(n->get_logger(), "EE link: %s", ee_link_param.c_str());
  RCLCPP_INFO(n->get_logger(), "Base link: %s", base_link_param.c_str());
  RCLCPP_INFO(n->get_logger(), "Arm ID: %s", arm_id_param.c_str());
  ee_link_name_ = ee_link_param;
  base_link_name_ = base_link_param;

  // Initialize controllers/solvers with provided arm_id
  this->jointVelocityController.initialize(n, arm_id_param);
  RCLCPP_INFO(n->get_logger(), "JointVelocityController initialized");

  // Initialize KDL solver from URDF (robot_description)
  if (!kdlSolver.initialize(n, arm_id_param)) {
    RCLCPP_ERROR(n->get_logger(), "Failed to initialize KDLSolver from robot_description");
  }

  // Initialize TF buffer/listener for EE pose lookups
  tf_buffer = std::make_shared<tf2_ros::Buffer>(n->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Parameters already read above; expose to HIROAvoidance via declared params (without hard-coded defaults)
  if (!n->has_parameter("ee_link")) {
    n->declare_parameter<std::string>("ee_link", ee_link_name_);
  }
  if (!n->has_parameter("base_link")) {
    n->declare_parameter<std::string>("base_link", base_link_name_);
  }

  setJointLimits(node_handle);  // Initialize limits and construct HIROAvoidance (which reads params)
  getControlPoints();           // Then use it to get control points
  createRosPubsSubs();

  // TODO: Set previous velocities to zero
  this->prevQDot.resize(7);
  for (int i = 0; i < 7; i++) {
    prevQDot(i) = 0;
  }
  // TODO: Set values used for PID control, can we do this somewhere else?
  this->cumulative_error = 0;
  this->derivative_error = 0;
  RCLCPP_INFO(n->get_logger(), "CartesianPositionController initialized successfully");
}

CartesianPositionController::CartesianPositionController(bool isSim = true) {
  this->isSim = isSim;
  jointVelocityController = JointVelocityController(this->isSim);
  getControlPoints();
  // setJointLimits(node_handle);
  createRosPubsSubs();
  prevQDot.resize(7);
  for (int i = 0; i < 7; i++) {
    prevQDot(i) = 0;
  }

  cumulative_error = 0;
  derivative_error = 0;
}
/*
    set the avoidance mode for the controller
*/
void CartesianPositionController::setAvoidanceMode(AvoidanceMode avoidanceModeName) {
    avoidanceMode = avoidanceModeName;
}


/*
    get the avoidance mode for the controller
*/
AvoidanceMode CartesianPositionController::getAvoidanceMode() {
    return avoidanceMode;
}

/*
    move to a desired position.
*/
void CartesianPositionController::moveToPosition(Eigen::Vector3d desiredPosition)
{
    bool inContact = false;

    // get error in position
    float current_vel = 0.0;
    positionError = desiredPosition - endEffectorPosition;
    RCLCPP_INFO(n->get_logger(), "Position error: %f, %f, %f", positionError(0), positionError(1), positionError(2));
    // loop until position error is sufficiently small
    while (positionError.norm() > positionErrorThreshold && rclcpp::ok()) {
        // get EE pos
        readEndEffectorPosition();
        // get pos of control points
        readControlPointPositions();
        if (inContact) {
            // reset current vel
            current_vel = 0.0;
        }
        joint_positions = kdlSolver.forwardKinematicsJoints(q);
        positionError = desiredPosition - endEffectorPosition;
        RCLCPP_INFO(n->get_logger(), "Position error: %f, %f, %f", positionError(0), positionError(1), positionError(2));
        if (positionError.norm() < 0.25)
        {
            desiredEEVelocity = positionError.norm() * positionError.normalized();

        } else {
            // restrict velocity to have a norm of 0.25
            current_vel = std::min(0.35, current_vel + 0.001);
            desiredEEVelocity = current_vel * positionError.normalized();
        }

        rclcpp::spin_some(n);

        switch (avoidanceMode)
        {
            case noAvoidance:
                // case of no avoidance -- convert x_dot to q_dot
                qDot = EEVelocityToQDot(desiredEEVelocity);
                // qDot = (0.3 * qDot) + (0.7 * prevQDot);
                jointVelocityController.sendVelocities(qDot);
                break;
            case HIRO:
            {
                // qDot = hiroAvoidance.computeJointVelocities(q, desiredEEVelocity,
                //                                             obstaclePositionVectors,
                //                                             closestPointsOnRobot, rate, endEffectorPosition, desiredPosition, arrow_publisher);
                EEVelocity = getEEVelocity();
                qDot = hiroAvoidance.computeJointVelocitiesWithExtCartForce(q, desiredEEVelocity,
                                                        externalCartesianForce,
                                                        externalCartesianWrench,
                                                        obstaclePositionVectors,
                                                        closestPointsOnRobot, rate, endEffectorPosition, desiredPosition, EEVelocity,
                                                        inContact);
                jointVelocityController.sendVelocities(qDot);
                break;
            }
        }
        rate.sleep();
    }
    // If no velocity is sent then the sim will continue at the last velocity sent
    if (this->isSim)
    {
        jointVelocityController.sendVelocities(Eigen::VectorXd::Constant(7, 0.0));
    }
}

/*
    move to position -- one step of it.
    Intended to be used inside of a control loop -- please refer to Main.cpp
    for an example.
*/
void CartesianPositionController::moveToPositionOneStepSeq(const Eigen::Vector3d desiredPositionVector, bool isEnd, double ee_rot)
{
    readEndEffectorPosition();
    Eigen::Vector3d EEVelocityVector = getEEVelocity();
    // can we further increase the p gain here?
    positionError = desiredPositionVector - endEffectorPosition;

    desiredEEVelocity = positionError / 0.01;
    desiredEEVelocity = positionError;
    double norm = desiredEEVelocity.norm();
    double velNorm = EEVelocityVector.norm();

    // Limit the desired velocity if it is too large
    if (robotInContact) {
        // start from low velocity
        // std::cout << norm << std::endl;
        current_vel_one_step = 0.0;

    }

    if(norm > 0.20 || !isEnd){
        current_vel_one_step = std::min(0.20, current_vel_one_step + 0.001);
        desiredEEVelocity = current_vel_one_step * desiredEEVelocity.normalized();
    } else {

        current_vel_one_step = std::min(norm, current_vel_one_step + 0.001);
        desiredEEVelocity = current_vel_one_step * desiredEEVelocity.normalized();
    }
    // test_pub.publish(current_vel_one_step);

    // else {
    //     desiredEEVelocity = positionError.normalized() * 0.2;
    // }

    rclcpp::spin_some(n);

    switch (avoidanceMode)
    {
        case noAvoidance:
            jointVelocityController.sendVelocities(EEVelocityToQDotRepulsive(desiredEEVelocity, obstaclePositionVectors));
            break;
        case HIRO:
        {
            // qDot = hiroAvoidance.computeJointVelocities(q, desiredEEVelocity,
            //                                             obstaclePositionVectors,
            //                                             closestPointsOnRobot, rate, endEffectorPosition, desiredPositionVector, arrow_publisher);
            // EEVelocity = getEEVelocity();
            qDot = hiroAvoidance.computeJointVelocitiesWithExtCartForce(q, desiredEEVelocity,
                                                        externalCartesianForce,
                                                        externalCartesianWrench,
                                                        obstaclePositionVectors,
                                                        closestPointsOnRobot, rate, endEffectorPosition, desiredPositionVector, EEVelocity,
                                                        robotInContact);


            // jointVelocityController.sendVelocities(qDot);
            break;
        }
    }
    // double threshold = 0.01;
    // std::cout << "ee_rot: " << ee_rot << "  qDot(6):   " << q(6) << std::endl;
    // if(abs(ee_rot - q(6)) > threshold){
    //     // if the error is negative, then
    //     if(ee_rot - q(6) < 0){
    //         qDot(6) = -0.3;
    //     }else{
    //         qDot(6) = 0.3;
    //     }
    // }else{
    //     qDot(6) = 0;
    // }
    jointVelocityController.sendVelocities(qDot);
    rate.sleep();
}

void CartesianPositionController::moveToPositionOneStep(const Eigen::Vector3d desiredPositionVector, bool isEnd)
{
    readEndEffectorPosition();
    Eigen::Vector3d EEVelocityVector = getEEVelocity();
    positionError = desiredPositionVector - endEffectorPosition;
    desiredEEVelocity = positionError;
    double norm = desiredEEVelocity.norm();
    double velNorm = EEVelocityVector.norm();

    // Limit the desired velocity if it is too large
    if (robotInContact) {
        current_vel_one_step = 0.0;
    }

    // Limit the robot's change in acceleration
    // TODO: Caleb - I don't think this is the best way to do this
    if(norm > 0.20 || !isEnd){
        current_vel_one_step = std::min(0.20, current_vel_one_step + 0.001);
        desiredEEVelocity = current_vel_one_step * desiredEEVelocity.normalized();
    } else {

        current_vel_one_step = std::min(norm, current_vel_one_step + 0.001);
        desiredEEVelocity = current_vel_one_step * desiredEEVelocity.normalized();
    }


    rclcpp::spin_some(n);


    const std::lock_guard<std::mutex> lock(mutex);
    joint_positions = kdlSolver.forwardKinematicsJoints(q);
    // RCLCPP_INFO(n->get_logger(), "Forward kinematics joint positions: %f, %f, %f, %f, %f, %f, %f", joint_positions(0), joint_positions(1), joint_positions(2), joint_positions(3), joint_positions(4), joint_positions(5), joint_positions(6));
    switch (avoidanceMode)
    {
        case noAvoidance:
            jointVelocityController.sendVelocities(EEVelocityToQDotRepulsive(desiredEEVelocity, obstaclePositionVectors));
            break;
        case HIRO:
        {
            qDot = hiroAvoidance.computeJointVelocities(q, desiredEEVelocity,
                                                    obstaclePositionVectors,
                                                    closestPointsOnRobot, rate, 
                                                    endEffectorPosition, desiredPositionVector, 
                                                    joint_positions);
            // EEVelocity = getEEVelocity();
            // qDot = hiroAvoidance.computeJointVelocitiesWithExtCartForce(q, desiredEEVelocity,
            //                                             externalCartesianForce,
            //                                             externalCartesianWrench,
            //                                             obstaclePositionVectors,
            //                                             closestPointsOnRobot, rate, endEffectorPosition, desiredPositionVector, EEVelocity,
                                                        // robotInContact);


            jointVelocityController.sendVelocities(qDot);
            break;
        }
    }
    rate.sleep();
}


/*
    command end effector velocity -- one step of it.
    Intended to be used inside of a control loop
*/
void CartesianPositionController::commandVelocityOneStep(const Eigen::Vector3d desiredEEVelocityVector, const Eigen::Vector3d desiredPositionVector)
{
    desiredEEVelocity = desiredEEVelocityVector;
    rclcpp::spin_some(n);
    joint_positions = kdlSolver.forwardKinematicsJoints(q);

    switch (avoidanceMode)
    {
        case noAvoidance:
        {
            // qDot = EEVelocityToQDot(desiredEEVelocity);
            qDot = EEVelocityToQDotRepulsive(desiredEEVelocity, obstaclePositionVectors);

            // moving average filter below
            for (int i = 0; i < 7; i++)
            {
                float total_sum = qDot(i);
                for (int j = 0; j < prevControlOutputs.size(); j++)
                {
                    total_sum += prevControlOutputs[j](i);
                }
                float joint_avg = total_sum / (prevControlOutputs.size() + 1);
                qDot(i) = joint_avg;
            }

            // if(this->xboxADown && this->xboxBDown){
            //     qDot(6) = 0;
            // }else if(this->xboxADown && q(6) < 2){
            //     qDot(6) = 0.3;
            // }else if(this->xboxBDown && q(6) > -2){
            //     qDot(6) = -0.3;
            // }else{
            //     qDot(6) = 0;
            // }
            qDot(6) = 0;
            // std::cout << prevQDot << std::endl;
            jointVelocityController.sendVelocities(qDot);
            break;
        }
        case HIRO:
        {
            // qDot = hiroAvoidance.computeJointVelocities(q, desiredEEVelocity,
            //                                             obstaclePositionVectors,
            //                                             closestPointsOnRobot, rate, endEffectorPosition, desiredPositionVector, arrow_publisher);
            // This is where I will send a new velocity that is commanded by the controllers a and b buttons
            EEVelocity = getEEVelocity();
            qDot = hiroAvoidance.computeJointVelocitiesWithExtCartForce(q, desiredEEVelocity,
                                                        externalCartesianForce,
                                                        externalCartesianWrench,
                                                        obstaclePositionVectors,
                                                        closestPointsOnRobot, rate, endEffectorPosition, desiredPositionVector, EEVelocity,
                                                        robotInContact);
            // std::cout << "last q: " << q(6) << std::endl;
            // if(this->xboxADown && this->xboxBDown){
            //     qDot(6) = 0;
            // }else if(this->xboxADown && q(6) < 2){
            //     qDot(6) = 0.3;
            // }else if(this->xboxBDown && q(6) > -2){
            //     qDot(6) = -0.3;
            // }else{
            //     qDot(6) = 0;
            // }
            qDot(6) = 0;
            jointVelocityController.sendVelocities(qDot);
            break;
        }
    }
    prevQDot = qDot;
    // restrict deque size to value defined in .h file
    if (prevControlOutputs.size() + 1 > maxPrevOutputsSize)
    {
        prevControlOutputs.pop_front();

    }
    prevControlOutputs.push_back(prevQDot);
    rate.sleep();
}



/*
    set joint velocities to 0.
*/
void CartesianPositionController::setVelocitiesToZero(){
    this->jointVelocityController.sendVelocities(Eigen::VectorXd::Constant(7, 0.0));
}

/*
    get and return the end effector position.
*/
Eigen::Vector3d CartesianPositionController::getEEPosition(){
    readEndEffectorPosition();
    return endEffectorPosition;
}

