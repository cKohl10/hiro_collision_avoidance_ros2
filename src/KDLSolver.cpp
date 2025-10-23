#include "KDLSolver.h"
#include "kdl/segment.hpp"
#include "kdl/frames.hpp"

/*
    KDL solver class.
    Includes various utilities for computing different jacobians
    and forward kinematics.
*/
KDLSolver::KDLSolver() {}

bool KDLSolver::initialize(const std::shared_ptr<rclcpp::Node> & node, const std::string & arm_id) {
    n = node;
    rclcpp::Logger logger = n ? n->get_logger() : rclcpp::get_logger("KDLSolver");

    if (!n) {
        RCLCPP_ERROR(logger, "Node handle is null in KDLSolver::initialize");
        return false;
    }

    // Read identifiers from parameters (no hard-coded link names)
    std::string arm_id_param;
    if (!n->has_parameter("arm_id")) {
        n->declare_parameter<std::string>("arm_id", "");
    }
    n->get_parameter("arm_id", arm_id_param);

    // Prefer parameter; fallback to provided argument if param missing
    arm_id_ = arm_id_param.empty() ? arm_id : arm_id_param;

    if (!n->has_parameter("base_link")) {
        n->declare_parameter<std::string>("base_link", "");
    }
    if (!n->has_parameter("ee_link")) {
        n->declare_parameter<std::string>("ee_link", "");
    }
    std::string base_link_param;
    std::string ee_link_param;
    n->get_parameter("base_link", base_link_param);
    n->get_parameter("ee_link", ee_link_param);
    if (base_link_param.empty() || ee_link_param.empty()) {
        RCLCPP_ERROR(logger, "Missing required parameters: base_link='%s' ee_link='%s'",
                     base_link_param.c_str(), ee_link_param.c_str());
        return false;
    }
    base_link_ = base_link_param;
    ee_link_ = ee_link_param;

    // Retrieve robot_description from this node's parameters
    if (!n->has_parameter("robot_description")) {
        n->declare_parameter<std::string>("robot_description", "");
    }
    n->get_parameter("robot_description", robot_desc_string);
    if (robot_desc_string.empty()) {
        RCLCPP_ERROR(logger, "robot_description parameter is empty; ensure launch passes it to this node");
        return false;
    }

    // Parse the URDF into a KDL::Tree
    if (!kdl_parser::treeFromString(robot_desc_string, kdlTree)) {
        RCLCPP_ERROR(logger, "Failed to construct KDL tree from robot_description");
        return false;
    }

    // Links are taken directly from parameters; do not add synthetic EE segments

    // Count control points in the tree
    controlPointCount = 0;
    for (KDL::SegmentMap::const_iterator seg = kdlTree.getSegments().begin(); seg != kdlTree.getSegments().end(); seg++)
    {
        if (seg->first.find("control_point") != std::string::npos)
        {
            controlPointCount++;
        }
    }

    // Build chains
    kdlChainsControlPoints = std::make_unique<KDL::Chain[]>(controlPointCount + 1);
    kdlChainsJoints = std::make_unique<KDL::Chain[]>(10); // 0 through 8 + ee

    kdlTree.getChain(base_link_, ee_link_, kdlChainsControlPoints[0]);
    for (int i = 0; i < controlPointCount; i++)
    {
        kdlTree.getChain(base_link_, std::string("control_point") + std::to_string(i), kdlChainsControlPoints[i+1]);
    }

    for (int i = 0; i < 9; i++)
    {
        kdlTree.getChain(base_link_, arm_id_ + "_link" + std::to_string(i), kdlChainsJoints[i]);
    }
    kdlTree.getChain(base_link_, ee_link_, kdlChainsJoints[9]);

    RCLCPP_INFO(logger, "KDL tree initialized successfully with %d control points", controlPointCount);
    return true;
}

/*
    computes the jacobian
    given the link name and q.
*/
Eigen::MatrixXd KDLSolver::computeJacobian(std::string linkName, Eigen::VectorXd q)
{
    // Read the element in the KDL chain array that needs to be indexed

    KDL::Chain kdlChain;
    bool success = kdlTree.getChain(base_link_, linkName, kdlChain);
    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("KDLSolver"),
                     "kdlTree.getChain failed. Verify that base_link ('%s') and tip ('%s') exist in robot_description",
                     base_link_.c_str(), linkName.c_str());
        // Emit a short list of known segment names to help debugging
        int printed = 0;
        for (const auto & seg : kdlTree.getSegments()) {
            if (printed >= 20) break; // avoid spamming
            RCLCPP_INFO(rclcpp::get_logger("KDLSolver"), "KDL segment: %s", seg.first.c_str());
            printed++;
        }
    }
    
    KDL::JntArray KDLJointArray(7);
    KDLJointArray.data = q;
    KDLJointArray.resize(kdlChain.getNrOfJoints());

    // joint to jacobian
    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChain);
    KDL::Jacobian J;
    J.resize(kdlChain.getNrOfJoints());
    // get jacobian, given current joint positions
    JSolver.JntToJac(KDLJointArray, J);
    return J.data;
}

/*
    computes the jacobian based on the control point and current joint positions, q
*/
Eigen::MatrixXd KDLSolver::computeJacobian2(KDLSolver::closest_point& controlPoint, Eigen::VectorXd& q)
{
    KDL::Chain kdlChain;
    std::string linkNameA = arm_id_ + "_link" + std::to_string(controlPoint.segmentId);
    Eigen::Vector3d ABin0 = controlPoint.t * (controlPoint.segmentPointB - controlPoint.segmentPointA);
    kdlTree.getChain(base_link_, linkNameA, kdlChain);
    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChain);
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChain.getNrOfJoints());
    KDL::Frame frame0A;
    FKSolver.JntToCart(KDLJointArray, frame0A);
    KDL::Frame newSegment(frame0A.M.Inverse() * KDL::Vector(ABin0.x(), ABin0.y(), ABin0.z()));
    kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), newSegment, KDL::RigidBodyInertia::Zero()));
    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChain);
    KDL::Jacobian J; J.resize(kdlChain.getNrOfJoints());
    JSolver.JntToJac(KDLJointArray, J);
    return J.data;
}

/*
    use forward kinematics for control points.
*/
Eigen::Vector3d KDLSolver::forwardKinematicsControlPoints(std::string controlPointName, Eigen::VectorXd q)
{
    int index{0};
    if (controlPointName == ee_link_)
    {
        index = 0;
    }
    else
    {
        index = std::stoi(controlPointName.substr(controlPointName.find_first_of("0123456789"), controlPointName.length() - 1)) + 1;
    }

    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChainsControlPoints[index]);
    KDL::Frame controlPointFrame;
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChainsControlPoints[index].getNrOfJoints());
    FKSolver.JntToCart(KDLJointArray, controlPointFrame);
    Eigen::Vector3d controlPointPositionVector;
    controlPointPositionVector << controlPointFrame.p(0), controlPointFrame.p(1), controlPointFrame.p(2);
    return controlPointPositionVector;
}

/*
    computation of forward kinematics based on joint configurations
*/
Eigen::MatrixXd KDLSolver::forwardKinematicsJoints(const Eigen::VectorXd & q)
{
    std::vector<int> jointNumbers = {2, 3, 4, 6, 7, 9};
    Eigen::MatrixXd result(3,jointNumbers.size());
    int index = 0;
    for (auto &&i : jointNumbers)
    {
        KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChainsJoints[i]);
        KDL::Frame kdlFrame;
        KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChainsJoints[i].getNrOfJoints());
        FKSolver.JntToCart(KDLJointArray, kdlFrame);
        result.col(index++) << kdlFrame.p(0), kdlFrame.p(1), kdlFrame.p(2);
    }

    return result;
}

/*
    forward kinematics, taking control point into account
*/
Eigen::Vector3d KDLSolver::forwardKinematics(KDLSolver::closest_point& controlPoint, Eigen::VectorXd& q)
{
    KDL::Chain kdlChain;
    std::string linkNameA = arm_id_ + "_link" + std::to_string(controlPoint.segmentId);
    Eigen::Vector3d ABin0 = controlPoint.t * (controlPoint.segmentPointB - controlPoint.segmentPointA);
    kdlTree.getChain(base_link_, linkNameA, kdlChain);
    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChain);
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChain.getNrOfJoints());
    KDL::Frame frame0A;
    FKSolver.JntToCart(KDLJointArray, frame0A);
    KDL::Frame newSegment(frame0A.M.Inverse() * KDL::Vector(ABin0.x(), ABin0.y(), ABin0.z()));
    kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), newSegment, KDL::RigidBodyInertia::Zero()));
    KDL::ChainFkSolverPos_recursive FKSolver2 = KDL::ChainFkSolverPos_recursive(kdlChain);
    KDL::Frame finalFrame;
    FKSolver2.JntToCart(KDLJointArray, finalFrame);
    return Eigen::Vector3d(finalFrame.p(0), finalFrame.p(1), finalFrame.p(2));
}

/*
    get the number of control points.
*/
int KDLSolver::getNumberControlPoints()
{
    return controlPointCount;
}

/*
    get the end effector link name.
*/
std::string KDLSolver::getEELink() const
{
    return ee_link_;
}
