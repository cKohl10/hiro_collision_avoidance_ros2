#include "utils/LoggingPublisher.h"

#include "hiro_collision_avoidance_msgs/msg/float_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float32.hpp"

/*
    constructor for logging publisher.
*/
LoggingPublisher::LoggingPublisher() {
  // Note: nh_ must be set before publishers can be created
  // This constructor creates a placeholder. Publishers should be initialized when node is available.
  RCLCPP_INFO(rclcpp::get_logger("LoggingPublisher"), "Creating logging publisher object");
  ee_path_msgarray_.data.resize(3);
  ee_traj_msgarray_.data.resize(4);
  seq_val_ = 0;
  
  // Publishers will be created when setNode() is called or in derived implementation
}

/*
    publishes end effector error.
*/
void LoggingPublisher::publishEE_Error(const double ee_error) {
  // RCLCPP_INFO(nh_->get_logger(), "Publishing end effector error...");
  if (!eeErrorStampedPublisher_) return;
  
  ee_error_msg_stamped_.data = ee_error;
  ee_error_msg_stamped_.header.stamp = nh_->now();
  // Note: ROS2 doesn't use seq in headers anymore
  
  ee_error_msg_.data = ee_error;
  // eeErrorPublisher_->publish(ee_error_msg);
  eeErrorStampedPublisher_->publish(ee_error_msg_stamped_);
  seq_val_++;
}

/*
    publish a couple of values:
        time parameter of the trajectory
        what was reached at that time parameter
*/
void LoggingPublisher::publishEETrajectory(const Eigen::Vector3d ee_path,
                                           const double time) {
  if (!eeTrajectoryPublisher_) return;
  
  for (int i = 0; i < 3; i++) {
    ee_traj_msgarray_.data[i] = ee_path[i];
  }

  ee_traj_msgarray_.data[3] = time;

  eeTrajectoryPublisher_->publish(ee_traj_msgarray_);
}

/*
    publish end-effector path.
*/
void LoggingPublisher::publishEEPath(const Eigen::Vector3d ee_path) {
  // RCLCPP_INFO(nh_->get_logger(), "Publishing end effector path...");
  if (!eePathPublisher_) return;
  
  for (int i = 0; i < ee_path.size(); i++) {
    ee_path_msgarray_.data[i] = ee_path[i];
  }
  eePathPublisher_->publish(ee_path_msgarray_);
}

LoggingPublisher::~LoggingPublisher() { 
  RCLCPP_INFO(rclcpp::get_logger("LoggingPublisher"), "Deleting LoggingPublisher"); 
}

void LoggingPublisher::publishRepulsiveVector(const Eigen::Vector3d& vec,
                                              const double& minDistance) {
  if (!repulsiveVectorPub_) return;
  
  geometry_msgs::msg::Vector3 vec_msg;
  vec_msg.x = vec[0];
  vec_msg.y = vec[1];
  vec_msg.z = vec[2];

  std_msgs::msg::Float32 minDist;
  minDist.data = minDistance;

  hiro_collision_avoidance_msgs::msg::RepulsiveVector repulse_msg;
  repulse_msg.vec = vec_msg;
  repulse_msg.min_dist = minDist;

  repulse_msg.header.stamp = nh_->now();
  repulse_msg.header.frame_id = "";
  repulsiveVectorPub_->publish(repulse_msg);
}

void LoggingPublisher::publishControlPointConstrains(
    const Eigen::VectorXd& boundlower, const Eigen::VectorXd& boundupper,
    const std::vector<Eigen::VectorXd>& boundlower_candidates,
    const std::vector<Eigen::VectorXd>& boundupper_candidates) {
  if (!controlPointConstraintsPub_) return;
  
  hiro_collision_avoidance_msgs::msg::ControlPointConstraints msg;
  for (std::size_t i = 0; i < boundlower.size(); i++) {
    msg.bl.push_back(boundlower[i]);
  }

  for (std::size_t i = 0; i < boundupper.size(); i++) {
    msg.bu.push_back(boundupper[i]);
  }

  std_msgs::msg::Float32MultiArray bl_cand_msg;
  for (std::size_t i = 0; i < boundlower_candidates.size(); i++) {
    Eigen::VectorXd vec = boundlower_candidates[i];
    for (std::size_t j = 0; j < vec.size(); j++) {
      bl_cand_msg.data.push_back(vec[j]);
    }
  }

  std_msgs::msg::Float32MultiArray bu_cand_msg;
  for (std::size_t i = 0; i < boundupper_candidates.size(); i++) {
    Eigen::VectorXd vec = boundupper_candidates[i];
    for (std::size_t j = 0; j < vec.size(); j++) {
      bu_cand_msg.data.push_back(vec[j]);
    }
  }

  msg.bl_candidates = bl_cand_msg;
  msg.bu_candidates = bu_cand_msg;

  msg.header.stamp = nh_->now();
  msg.header.frame_id = "";
  controlPointConstraintsPub_->publish(msg);
}

void LoggingPublisher::publishControlPoints(
    const std::vector<KDLSolver::closest_point>& controlPoints) {
  if (!controlPointsPub_) return;
  
  hiro_collision_avoidance_msgs::msg::PointArrayStamped msg;
  for (std::size_t i = 0; i < controlPoints.size(); i++) {
    Eigen::Vector3d pt_vec = controlPoints[i].control_point;
    geometry_msgs::msg::Point pt_msg;
    pt_msg.x = pt_vec[0];
    pt_msg.y = pt_vec[1];
    pt_msg.z = pt_vec[2];
    msg.points.push_back(pt_msg);
  }

  msg.header.stamp = nh_->now();
  msg.header.frame_id = "";
  controlPointsPub_->publish(msg);
}

void LoggingPublisher::addEEasControlPoint(
    const Eigen::MatrixXd& J, const Eigen::MatrixXd& fkControlPointsAtq,
    std::vector<Eigen::MatrixXd>& controlPointsJc,
    std::vector<KDLSolver::closest_point>& controlPoints) {
  controlPointsJc.emplace_back(J);
  Eigen::Vector3d pos = fkControlPointsAtq.col(fkControlPointsAtq.cols() - 1);
  KDLSolver::closest_point eeControlPoint;
  eeControlPoint.control_point = pos;
  controlPoints.emplace_back(eeControlPoint);
}

void LoggingPublisher::publishManipulabilityAnalysis(
    const std::vector<Eigen::MatrixXd>& Jc,
    const std::vector<KDLSolver::closest_point>& controlPoints,
    const Eigen::VectorXd& jointVelocityMax) {
  // Reading material: Comments on Manipulability Measure in Redundant Planar
  // Manipulators by Martins
  // https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-4-manipulability/
  // https://robotacademy.net.au/lesson/velocity-ellipsoid-in-3d-and-manipulability/

  if (!manipulabilityMetricsPub_) return;
  
  if (Jc.size() > controlPoints.size()) {
    RCLCPP_ERROR(nh_->get_logger(), "Invalid inputs. Skipping manipulability analysis.");
    return;
  }

  hiro_collision_avoidance_msgs::msg::ManipulabilityMetrics msg;
  for (std::size_t i = 0; i < Jc.size(); ++i) {
    // ROS_INFO_STREAM("Constraint point number: " << i);
    Eigen::MatrixXd j_contact = Jc[i];
    std::cout << "j_contact:\n" << j_contact << std::endl;

    if (j_contact.size() == 0) {
      RCLCPP_ERROR(nh_->get_logger(), "Jacobian is empty; skipping manipulability analysis.");
      continue;
    }

    // Here, we can optionally scale the matrix wrt to the max joint velocities
    // Eigen::MatrixXd jointVelMaxDiagMat =
    // jointVelocityMax.matrix().asDiagonal(); jointVelMaxDiagMat =
    // jointVelMaxDiagMat.block(0, 0, j_contact.cols(),
    //                                               jointVelMaxDiagMat.rows());
    // std::cout << "jointVelMaxDiagMat\n" << jointVelMaxDiagMat << std::endl;
    // j_contact = j_contact * jointVelMaxDiagMat;

    j_contact = j_contact.block(0, 0, 3, j_contact.cols());
    Eigen::MatrixXd j_t = j_contact.transpose();
    Eigen::MatrixXd j_sq = j_contact * j_t;
    std::cout << "j_sq:\n" << j_sq << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(j_sq, Eigen::ComputeThinU);
    std::cout << "Its singular values are:\n"
              << svd.singularValues() << std::endl;
    // std::cout << "Its left singular vectors are the columns of the
    //     thin U matrix :\n " << svd.matrixU() << std::endl;
    // std::cout << "Its right singular vectors are the columns of the thin
    // V matrix:\n" << svd.matrixV() << std::endl;

    std::vector<double> eigenvalues;
    for (std::size_t j = 0; j < svd.singularValues().size(); ++j) {
      msg.ellipse_dimensions.push_back(svd.singularValues()[j]);
      eigenvalues.emplace_back(svd.singularValues()[j]);
    }

    for (std::size_t j = 0; j < svd.matrixU().cols(); ++j) {
      for (std::size_t k = 0; k < svd.matrixU().col(0).size(); ++k) {
        double eigen_vec_val = svd.matrixU().col(j)[k];
        msg.ellipse_axis_vectors.push_back(eigen_vec_val);
      }
    }

    // Below we have different way to compute eigenvalues and eigenvectors
    // Yields the same results. Great for verification.

    // Eigen::EigenSolver<Eigen::MatrixXd> es(j_sq);
    // std::cout << "The eigenvalues of A are:\n" << std::endl
    //           << es.eigenvalues() << std::endl;
    // std::cout << "The matrix of eigenvectors, V, is:\n" << std::endl
    //           << es.eigenvectors() << std::endl;

    // for (std::size_t j = 0; j < es.eigenvalues().rows(); ++j) {
    //   double eigenval = es.eigenvalues().col(0)[j].real();
    // //   std::cout << "eigenval: " << eigenval << std::endl;
    //   msg.ellipse_dimensions.push_back(sqrt(eigenval));
    // }

    // for (std::size_t j = 0; j < es.eigenvectors().cols(); ++j) {
    //   for (std::size_t k = 0; k < es.eigenvectors().col(0).size(); ++k) {
    //     double eigen_comp_real = es.eigenvectors().col(j)[k].real();
    //     msg.ellipse_axis_vectors.push_back(eigen_comp_real);
    //   }
    // }

    double j_det = j_sq.determinant();
    std::cout << "j_det: " << j_det << std::endl;
    double manip_val = sqrt(abs(j_det));
    msg.yoshikawa_scalar.push_back(manip_val);
    std::cout << "manip_val: " << manip_val << std::endl;

    double min_eig = *std::min_element(eigenvalues.begin(), eigenvalues.end());
    double max_eig = *std::max_element(eigenvalues.begin(), eigenvalues.end());
    double ratio = max_eig / min_eig;
    std::cout << "ratio: " << ratio << std::endl;

    Eigen::Vector3d pt_vec = controlPoints[i].control_point;
    geometry_msgs::msg::Point pt_msg;
    pt_msg.x = pt_vec[0];
    pt_msg.y = pt_vec[1];
    pt_msg.z = pt_vec[2];
    msg.points.push_back(pt_msg);
  }

  //   std::cout << "msg.points.size(): " << msg.points.size() << std::endl;
  //   std::cout << "msg.ellipse_dimensions.size(): " <<
  //   msg.ellipse_dimensions.size() << std::endl; std::cout <<
  //   "msg.ellipse_axis_vectors.size(): " << msg.ellipse_axis_vectors.size() <<
  //   std::endl;

  msg.header.stamp = nh_->now();
  msg.header.frame_id = "";
  manipulabilityMetricsPub_->publish(msg);
}

  void LoggingPublisher::publishArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const int& id){
        // TODO: Caleb - use a custom message type in this function
        // The published arrow is picked up by arrow_visualizer.py you must run that file
        if (!arrowPub_) return;
        
        std_msgs::msg::Float64MultiArray msg;
        std::vector<double> data;

        // arrow start point    
        data.push_back(start(0));
        data.push_back(start(1));
        data.push_back(start(2));

        // arrow end point
        data.push_back(end(0));
        data.push_back(end(1));
        data.push_back(end(2));

        data.push_back(id);
        
        msg.data = data;
        this->arrowPub_->publish(msg);
}