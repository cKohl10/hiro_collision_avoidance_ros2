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

    // Construct link names based on arm_id and store as member variables
    arm_id_ = arm_id;
    base_link_ = arm_id_ + "_link0";
    std::string link8 = arm_id_ + "_link8";
    ee_link_ = arm_id_ + "_EE";

    // Add fixed EE offset segment (matches ROS1 behavior)
    KDL::Frame newSegment(KDL::Vector(0, 0, 0.1034));
    kdlTree.addSegment(KDL::Segment(ee_link_, KDL::Joint(KDL::Joint::None), newSegment, KDL::RigidBodyInertia::Zero()), link8);

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
    kdlTree.getChain(base_link_, linkName, kdlChain);

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
