#include "KDLSolver.h"
#include "kdl/segment.hpp"
#include "kdl/frames.hpp"

/*
    KDL solver class.
    Includes various utilities for computing different jacobians
    and forward kinematics.
*/
KDLSolver::KDLSolver() {
    // Note: node 'n' must be initialized before calling this constructor
    if (n) {
        n->declare_parameter("robot_description", "");
        n->get_parameter("robot_description", robot_desc_string);
    }
    if (!kdl_parser::treeFromString(robot_desc_string, kdlTree))
        RCLCPP_ERROR(rclcpp::get_logger("KDLSolver"), "Failed to construct kdl tree");
    // why is this value hardcoded as 0.1034?
    KDL::Frame newSegment(KDL::Vector(0, 0, 0.1034));
    kdlTree.addSegment(KDL::Segment("panda_EE", KDL::Joint(KDL::Joint::None), newSegment, KDL::RigidBodyInertia::Zero()), "panda_link8");

    for (KDL::SegmentMap::const_iterator seg = kdlTree.getSegments().begin(); seg != kdlTree.getSegments().end(); seg++)
    {
        if (seg->first.find("control_point") != std::string::npos)
        {
            controlPointCount++;
        }
    }

    kdlChainsControlPoints = std::make_unique<KDL::Chain[]>(controlPointCount + 1);
    kdlChainsJoints = std::make_unique<KDL::Chain[]>(10); // 0 through 8 + ee

    kdlTree.getChain("panda_link0", "panda_EE", kdlChainsControlPoints[0]);
    for (int i = 0; i < controlPointCount; i++)
    {
        kdlTree.getChain("panda_link0", std::string("control_point") + std::to_string(i), kdlChainsControlPoints[i+1]);
    }

    for (int i = 0; i < 9; i++)
    {
        kdlTree.getChain("panda_link0", std::string("panda_link") + std::to_string(i), kdlChainsJoints[i]);
    }
    kdlTree.getChain("panda_link0", std::string("panda_EE"), kdlChainsJoints[9]);
}

/*
    computes the jacobian
    given the link name and q.
*/
Eigen::MatrixXd KDLSolver::computeJacobian(std::string linkName, Eigen::VectorXd q)
{
    // Read the element in the KDL chain array that needs to be indexed

    KDL::Chain kdlChain;
    kdlTree.getChain("panda_link0", linkName, kdlChain);

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
    std::string linkNameA = std::string("panda_link") + std::to_string(controlPoint.segmentId);
    Eigen::Vector3d ABin0 = controlPoint.t * (controlPoint.segmentPointB - controlPoint.segmentPointA);
    kdlTree.getChain("panda_link0", linkNameA, kdlChain);
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
    if (controlPointName == "panda_EE")
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
    std::string linkNameA = std::string("panda_link") + std::to_string(controlPoint.segmentId);
    Eigen::Vector3d ABin0 = controlPoint.t * (controlPoint.segmentPointB - controlPoint.segmentPointA);
    kdlTree.getChain("panda_link0", linkNameA, kdlChain);
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
