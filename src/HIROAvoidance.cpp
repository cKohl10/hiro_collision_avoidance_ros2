// Link to Ding Paper Presentation:
// https://docs.google.com/presentation/d/1LrW7mna1wRgHsIzw3wXOrvIg3xlkNpIfmVRfGyxG_v0/edit?usp=sharing

// Link to Overleaf file with the math in the presentation
// https://www.overleaf.com/read/hwndqxxqtvds

#include "HIROAvoidance.h"
#include <tuple>



/*
    hiro avoidance constructor
*/
HIROAvoidance::HIROAvoidance(){
    // set cartesian compliance matrix
    compliance_gains.resize(3);
    compliance_gains << 0.03, 0.03, 0.03;
    setCartesianComplianceGains(compliance_gains);
    forceBasedRepulsiveVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);

}

HIROAvoidance::HIROAvoidance(JointLimits jointLimits, std::shared_ptr<rclcpp::Node> node_handle){
    _jointLimits = jointLimits;
    compliance_gains.resize(3);
    compliance_gains << 0.03, 0.03, 0.03;

    setCartesianComplianceGains(compliance_gains);
    forceBasedRepulsiveVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->obstacle_life = 0;
    this->hit_life = 0;

    // Get parameters (they should already be declared by CartesianPositionController)
    ee_link_name = node_handle->get_parameter("ee_link").as_string();
    base_link_name = node_handle->get_parameter("base_link").as_string();

    // Initialize our internal KDL solver using the same parameters so base/tip are valid
    std::string arm_id_param;
    if (!node_handle->has_parameter("arm_id")) {
        node_handle->declare_parameter<std::string>("arm_id", "");
    }
    node_handle->get_parameter("arm_id", arm_id_param);
    if (arm_id_param.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("HIROAvoidance"), "Parameter 'arm_id' is empty; KDLSolver initialization may fail");
    }
    if (!kdlSolver.initialize(node_handle, arm_id_param)) {
        RCLCPP_ERROR(rclcpp::get_logger("HIROAvoidance"), "Failed to initialize KDLSolver inside HIROAvoidance");
    }
}

void HIROAvoidance::setCartesianComplianceGains(Eigen::VectorXd gains) {
    cartesianComplianceMatrix = Eigen::MatrixXd::Identity(3, 3);
    cartesianComplianceMatrix(0,0) = gains(0);
    cartesianComplianceMatrix(1,1) = gains(1);
    cartesianComplianceMatrix(2,2) = gains(2);
}

/*
    v2 of computing damping factor -- from the paper:

    Task Space Control of Articulated Robot Near
    Kinematic Singularity: Forward Dynamics Approach

    this value is expected to help the manipulator avoid kinematic singularities;
    namely, when the determinant approaches 0.
*/
double HIROAvoidance::computeDampingFactorV2(double b_0, double a, Eigen::MatrixXd jacobian)
{

    double determinant = jacobian.determinant();
    double determinant_squared = determinant * determinant;
    // TODO(peasant98) -- use better variable naming
    return b_0 * std::exp(determinant_squared / (a * a));

}

/*
    compute the damping factor, parameterized by omega.
*/
double HIROAvoidance::computeDampingFactor(double omega)
{
    if (omega >= _omega0)
    {
        return 0.0;
    }
    else
    {
        return _dampingFactor0 * std::pow((1 - omega/_omega0),2);
    }
}


// Public Functions


void HIROAvoidance::getVelocityLimits(Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound,
                                                 Eigen::VectorXd& q, rclcpp::Rate& r){
    Eigen::Vector3d candidates;
    std::vector<Eigen::VectorXd> lowerBoundCandidates{std::size_t(_jointLimits.jointLimitsMax.size()), Eigen::VectorXd{_jointLimits.jointLimitsMax.size()}};
    std::vector<Eigen::VectorXd> upperBoundCandidates{std::size_t(_jointLimits.jointLimitsMax.size()), Eigen::VectorXd{_jointLimits.jointLimitsMax.size()}};

    // Note: This code differs from flacco in that there is no body restriction option
    double cycle_time = std::chrono::duration<double>(r.period()).count();
    for (int i = 0; i < _jointLimits.jointLimitsMax.size(); i++)
    {
        candidates(0) = (_jointLimits.jointLimitsMin(i) - q(i)) / cycle_time;
        candidates(1) = - _jointLimits.jointVelocityMax(i);
        candidates(2) = - std::sqrt(2 * _jointLimits.jointAccelerationMax(i) * (q(i) - _jointLimits.jointLimitsMin(i)));
        lowerBound(i) = candidates.maxCoeff();
        lowerBoundCandidates[i] = candidates;

        candidates(0) = (_jointLimits.jointLimitsMax(i) - q(i)) / cycle_time;
        candidates(1) = + _jointLimits.jointVelocityMax(i);
        candidates(2) = + std::sqrt(2 * _jointLimits.jointAccelerationMax(i) * (q(i) - _jointLimits.jointLimitsMin(i)));
        upperBoundCandidates[i] = candidates;
        upperBound(i) = candidates.minCoeff();
    }
    // Note: Logging removed - was: loggingPublisher_.publishControlPointConstrains(...)
}

Eigen::MatrixXd HIROAvoidance::getHMatrix(Eigen::VectorXd& q, Eigen::Vector3d& xDot, bool drop_goal)
{
        if (!drop_goal){
        double xDot_norm = xDot.norm();
        double secondaryTaskGain = 0.01;
        double time_delta = 1.0/10.0;
        Eigen::MatrixXd m_squared = _jointLimits.jointRanges.cwiseQuotient(_jointLimits.jointRanges);
        Eigen::MatrixXd i_div(7, 7);
        i_div << 1/(2*m_squared(0)), 0, 0, 0, 0, 0, 0,
                0, 1/(2*m_squared(1)), 0, 0, 0, 0, 0,
                0, 0, 1/(2*m_squared(2)), 0, 0, 0, 0,
                0, 0, 0, 1/(2*m_squared(3)), 0, 0, 0,
                0, 0, 0, 0, 1/(2*m_squared(4)), 0, 0,
                0, 0, 0, 0, 0, 1/(2*m_squared(5)), 0,
                0, 0, 0, 0, 0, 0, 1/(2*m_squared(6));
        Eigen::MatrixXd middle_joint_H_term = (secondaryTaskGain * i_div * time_delta * time_delta);

        Eigen::MatrixXd J = kdlSolver.computeJacobian(ee_link_name, q).block(0,0,3,7);
        Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd H = J.transpose() * J + computeDampingFactor(std::sqrt((J*J.transpose()).determinant())) * Eigen::MatrixXd::Identity(7,7) + middle_joint_H_term;

        // example of how to use the v2 damping factor function
        // Eigen::MatrixXd H = J.transpose() * J + computeDampingFactorV2(1.0, 1.0, J) * Eigen::MatrixXd::Identity(7,7) + middle_joint_H_term;

        return H;
    } else{
        Eigen::MatrixXd J = kdlSolver.computeJacobian(ee_link_name, q).block(0,0,3,7);

        Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd qGroundTruth = Jpinv * xDot ;

        Eigen::MatrixXd H = J.transpose() * J + computeDampingFactor(std::sqrt((J*J.transpose()).determinant())) * Eigen::MatrixXd::Identity(7,7);
        // example of how to use the v2 damping factor function
        // Eigen::MatrixXd H = J.transpose() * J + computeDampingFactorV2(1.0, 1.0, J) * Eigen::MatrixXd::Identity(7,7);
        return H;
    }
}

Eigen::VectorXd HIROAvoidance::getfVector(Eigen::VectorXd& q, Eigen::Vector3d& xDot, bool drop_goal){

    if (!drop_goal){
        double xDot_norm = xDot.norm();
        double secondaryTaskGain = 0.0001;
        double time_delta = 1.0/1.0;
        Eigen::MatrixXd m_squared = _jointLimits.jointRanges.cwiseQuotient(_jointLimits.jointRanges);
        Eigen::MatrixXd middle_joint_f_term_1 = (secondaryTaskGain * q.cwiseQuotient(m_squared).transpose() * time_delta);
        Eigen::MatrixXd middle_joint_f_term_2 = -(secondaryTaskGain * _jointLimits.jointMiddleValues.cwiseQuotient(m_squared).transpose() * time_delta);

        Eigen::MatrixXd J = kdlSolver.computeJacobian(ee_link_name, q).block(0,0,3,7);
        Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::VectorXd f = - xDot.transpose() * J + middle_joint_f_term_1 + middle_joint_f_term_2;
        return f;

    } else{
        Eigen::MatrixXd J = kdlSolver.computeJacobian(ee_link_name, q).block(0,0,3,7);

        Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::VectorXd f = - xDot.transpose() * J;
        return f;

    }
}


Eigen::VectorXd HIROAvoidance::computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d xDot,
                                                    std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                    const std::vector<KDLSolver::closest_point>& constClosestPoints,
                                                    rclcpp::Rate& r, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_goal,
                                                    Eigen::MatrixXd& jointPositions)
{

    std::vector<KDLSolver::closest_point> closestPoints = constClosestPoints;
    drop_goal = false;
    // Just a couple of variables in here I think we might need
    resetComputationObjects();

    Eigen::VectorXd lower_bound{_jointLimits.jointLimitsMax.size()}, upper_bound{_jointLimits.jointLimitsMax.size()};
    // bool drop_goal = false;
    // bool drop_goal = alter_ee_path(xDot, obstaclePositionVectors, ee_pos, ee_goal, arrow_publisher, closestPoints);
    Eigen::Vector3d ee_o_error;
    Eigen::Vector3d ctrl_point_o_error;

    Eigen::Vector3d rev;
    Eigen::Vector3d end_pos;
    double ee_o_mag;
    double ctrl_point_o_mag;
    double rev_mag;

    alterEEVelocityFromObstacles(xDot, obstaclePositionVectors, ee_pos);
    getVelocityLimits(lower_bound, upper_bound, q, r);

    // construct h matrix and f vector for qp formulation
    Eigen::MatrixXd H = getHMatrix(q, xDot, drop_goal);
    Eigen::VectorXd f = getfVector(q, xDot, drop_goal);


    Eigen::MatrixXd A, newA;
    Eigen::VectorXd b, newb;
    std::vector<Eigen::MatrixXd> controlPointsJc(closestPoints.size());

    // create all restrictions/constraints below
    if (obstaclePositionVectors.size() == 0){
        // No restrictions placed on movement, b/c there's no obs!
        A = Eigen::MatrixXd(0, 0);
        b = Eigen::VectorXd(0);
    } else {
        // Add restrictions for every obstacle
        int m = obstaclePositionVectors.size();
        A = Eigen::MatrixXd(m, 7);
        b = Eigen::VectorXd(m);
        Eigen::MatrixXd Ji = Eigen::MatrixXd::Zero(3, 7);
        Eigen::MatrixXd JiResized = Eigen::MatrixXd::Zero(3, 7);

        // algorithm below:

        // 1) select the control point closest to each obstacle
        // 2) Calculate the distance d from the obstacle point to chosen control point
        // 3) Get Jacobian to selected control point. This is changing is size dependent on which control point we've selected
        // 4) Alter the size of the Jacobian so that it's always 7x3
        // comment (from matt): what and where is equation 5? and fig 5?
        // 5) Add extra row from equation 5
        // 6) d norm calculation done in Fig. 5
        // 7) Take the gradient of the norm distance with
        int numberOfRestrictions = 0;
        for (int i = 0; i < obstaclePositionVectors.size(); i++){
            JiResized = Eigen::MatrixXd::Zero(3, 7);
            b[i] = computebvalue(closestPoints[i].distance_to_obs); // From fig. 5
            if (!std::isnan(b(i))) numberOfRestrictions++;
            Ji = kdlSolver.computeJacobian2(closestPoints[i], q);
            JiResized.block(0, 0, 3, Ji.cols()) = Ji.block(0, 0, 3, Ji.cols());
            controlPointsJc[i] = JiResized;
            A.row(i) = (obstaclePositionVectors[i] - closestPoints[i].control_point).normalized().transpose() * JiResized;
        }

        newA.resize(numberOfRestrictions, A.cols());
        newb.resize(numberOfRestrictions);
        int j = 0;
        // loop through amt of obstacles
        for (int i = 0; i < m; i++){
            if (!std::isnan(b(i))){
                // if the obstacle is close, update the new A matrix
                newA.row(j) = A.row(i);
                // update the new b vector
                newb(j) = b(i);
                // increment for the A matrix, b vectors
                j++;
            }
        }
    }

    // Note: Logging removed - was: loggingPublisher_.publishControlPoints(...)
    // Note: Logging removed - was: loggingPublisher_.addEEasControlPoint(...)
    // Note: Logging removed - was: loggingPublisher_.publishManipulabilityAnalysis(...)

    // run optimization to return joint velocities
    return algLib(H, f, newA, newb, lower_bound, upper_bound);
}
bool HIROAvoidance::isCloseToZero(double value, double threshold) {
        return (0.0 - threshold <= value <= 0.0 + threshold);
    }
void HIROAvoidance::resetComputationObjects(){
    new_obs = false;
    x_dists_pos.clear();
    y_dists_pos.clear();
    z_dists_pos.clear();
    x_dists_neg.clear();
    y_dists_neg.clear();
    z_dists_neg.clear();
    q_dot = Eigen::VectorXd::Zero(7);

    //Variables used in collision thresholding
    min_x_dist = 0;
    max_x_dist = 0;
    x_dist_pos_dif = 0;
    x_dist_neg_dif = 0;

    min_y_dist = 0;
    max_y_dist = 0;
    y_dist_pos_dif = 0;
    y_dist_neg_dif = 0;

    min_z_dist = 0;
    max_z_dist = 0;
    z_dist_pos_dif = 0;
    z_dist_neg_dif = 0;

    x_mean = signal_parser_x.current_mean;
    y_mean = signal_parser_y.current_mean;
    // std::cout << "zcurr:  " << signal_parser_z.current_mean;
    z_mean = signal_parser_z.current_mean;

    x_std_dev = signal_parser_x.current_stdDev;
    y_std_dev = signal_parser_y.current_stdDev;
    z_std_dev = signal_parser_z.current_stdDev;

    added_x_std_dev = std::min(x_std_dev/max_std_dev*max_std_dev_dif, max_std_dev_dif);
    added_y_std_dev = std::min(y_std_dev/max_std_dev*max_std_dev_dif, max_std_dev_dif);
    added_z_std_dev = std::min(z_std_dev/max_std_dev*max_std_dev_dif, max_std_dev_dif);

    x_contact = false;
    y_contact = false;
    z_contact = false;
}

void HIROAvoidance::alterEEVelocityFromObstacles(Eigen::Vector3d & xDot, std::vector<Eigen::Vector3d>& obstaclePositionVectors, Eigen::Vector3d& ee_pos){
    Eigen::Vector3d rev;
    Eigen::Vector3d ee_o_error;
    Eigen::Vector3d end_pos;
    double ee_o_mag;
    double rev_mag;

    // Adds a repulsive velocity to the cartesian velocity
    for (int i = 0; i < obstaclePositionVectors.size(); i++){
        
        ee_o_error = obstaclePositionVectors[i] - ee_pos;
        ee_o_mag = ee_o_error.norm();


        if (ee_o_mag <= 0.5){
            
            if(ee_o_error(0) < 0){
                x_dists_neg.push_back(ee_o_error(0));
            }else{
                x_dists_pos.push_back(ee_o_error(0));
            }

            if(ee_o_error(1) < 0){
                y_dists_neg.push_back(ee_o_error(1));
            }else{
                y_dists_pos.push_back(ee_o_error(1));
            }

            if(ee_o_error(2) < 0){
                z_dists_neg.push_back(ee_o_error(2));
            }else{
                z_dists_pos.push_back(ee_o_error(2));
            }

            rev = -ee_o_error;
            // TODO: Take this out, it is not a good way to scale this
            rev_mag = rev.norm();
            rev = rev/rev_mag;
            end_pos = rev - ee_pos;
            // Stops the robot completely when someing is close to the EE
            if (rev_mag < 0.08){
                rev_mag *= 0;
            }
            xDot(1) = xDot(1) * (std::abs(rev_mag)/0.8) + rev(1)*1.5*(0.5-ee_o_mag);
            xDot(2) = xDot(2) * (std::abs(rev_mag)/0.8) + rev(2)*0.2*(0.5-ee_o_mag);
            xDot(0) = xDot(0) * (std::abs(rev_mag)/0.8) + rev(0)*1.5*(0.5-ee_o_mag);
            obstacle_life = 1;
            force_memory = (std::abs(rev_mag)/0.8);
            new_obs = true;
        }
    }
}

void HIROAvoidance::updateExistingObstacleEEForce(Eigen::Vector3d & xDot){
    if(!new_obs){
        if(hit_life != 0){
            obstacle_life = 0;
        }
        if(obstacle_life != 0){
            obstacle_life++;
            xDot(1) = xDot(1) * (force_memory + (1.0 - force_memory) * (obstacle_life/max_life));
            xDot(2) = xDot(2) * (force_memory + (1.0 - force_memory) * (obstacle_life/max_life));
            xDot(0) = xDot(0) * (force_memory + (1.0 - force_memory) * (obstacle_life/max_life));
        }
        if(obstacle_life > max_life){
            obstacle_life = 0;
        }
    }
}
void HIROAvoidance::updateExistingObstacleThresholdReduction(){
    //TODO: need to add this computation for all directions and axes
    if(y_dist_neg_dif != 0){
        slow_down_memory_y = y_dist_neg_dif;
        life_memory_y = 1;
    }
    if(y_dist_neg_dif == 0 && life_memory_y != 0){
        life_memory_y++;
        y_dist_neg_dif = (slow_down_memory_y - slow_down_memory_y * (life_memory_y/max_life));
        if(life_memory_y > max_life){
            life_memory_y = 0;
        }
    }
}
void HIROAvoidance::updateExistingContactForce(Eigen::Vector3d & xDot, bool & inContact){
    // std::cout << hit_life << std::endl;
    if(hit_life != 0){
        hit_life++;
        inContact = true;
        xDot(0) = ((force_hit*ext_vel(0)) * (1.0 - hit_life/max_life));
        xDot(1) = ((force_hit*ext_vel(1)) * (1.0 - hit_life/max_life));
        xDot(2) = ((force_hit*ext_vel(2)) * (1.0 - hit_life/max_life));
        if(hit_life > max_life){
            hit_life = 0;
        }
    } else {
        inContact = false;
    }
}

void HIROAvoidance::checkForContact(Eigen::Vector3d & externalWrench){
    x_neg_thresh = x_mean - min_newton_dif - added_x_std_dev + x_dist_neg_dif;
    x_pos_thresh = x_mean + min_newton_dif + added_x_std_dev - x_dist_pos_dif;
    y_neg_thresh = y_mean - min_newton_dif - added_y_std_dev + y_dist_neg_dif;
    y_pos_thresh = y_mean + min_newton_dif + added_y_std_dev - y_dist_pos_dif;
    // std::cout << "z_mean  " << z_mean << std::endl;
    // std::cout << "z_dist  " << z_dist_neg_dif << std::endl;
    // std::cout << "z_dist  " << added_z_std_dev << std::endl;
    z_neg_thresh = z_mean - min_newton_dif - added_z_std_dev + z_dist_neg_dif;
    z_pos_thresh = z_mean + min_newton_dif + added_z_std_dev - z_dist_pos_dif;

    //TODO: This is for visualization only, can be removed
    signal_parser_x_upper.getSignal(x_neg_thresh);
    signal_parser_x_lower.getSignal(x_pos_thresh);

    signal_parser_y_upper.getSignal(y_neg_thresh);
    signal_parser_y_lower.getSignal(y_pos_thresh);

    signal_parser_z_upper.getSignal(z_neg_thresh);
    signal_parser_z_lower.getSignal(z_pos_thresh);


    x_contact = (externalWrench(0) < x_neg_thresh || externalWrench(0) > x_pos_thresh);
    y_contact = (externalWrench(1) < y_neg_thresh || externalWrench(1) > y_pos_thresh);
    // std::cout << "zWrench:  " << externalWrench(2) << std::endl;
    // std::cout << "z_neg_thresh:  " << z_neg_thresh << std::endl;
    // std::cout << "z_pos_thresh:  " << z_pos_thresh << std::endl;
    z_contact = (externalWrench(2) < z_neg_thresh || externalWrench(2) > z_pos_thresh);
}
void HIROAvoidance::applyContactBehavior(Eigen::Vector3d & externalWrench){
    // std::cout << y_contact << x_contact << z_contact << std::endl;
    // if(y_contact || x_contact|| z_contact){
    if(y_contact || x_contact){
        hit_life = 1;
        force_hit = -1;
        Eigen::Vector3d ext_force(externalWrench(0) - x_mean,
                                  externalWrench(1) - y_mean,
                                  0);
                                //   externalWrench(2) - z_mean);
        double norm = ext_force.norm();

        for (int c = 0; c < 3; c++)
        {
            ext_force(c) = std::max(std::min(ext_force(c), 30.0), -30.0);
        }

        ext_vel = cartesianComplianceMatrix * ext_force;
        for (int c = 0; c < 3; c++)
        {
            ext_vel(c) = std::max(std::min(ext_vel(c), 0.3), -0.3);
        }
        Eigen::Vector3d normalized_ext_force = ext_force / norm;
    }
}
void HIROAvoidance::computeObstacleThresholdReduction(){
    if (x_dists_pos.size() > 0){
        max_x_dist = *min_element(x_dists_pos.begin(), x_dists_pos.end());
        x_dist_pos_dif = ((max_dist - max_x_dist)/(max_dist - min_dist)) * dist_max_reduction;
    }

    if (x_dists_neg.size() > 0){
        min_x_dist = *max_element(x_dists_neg.begin(), x_dists_neg.end());
        x_dist_neg_dif = ((max_dist - std::abs(min_x_dist))/(max_dist - min_dist)) * dist_max_reduction;
    }


    if (y_dists_pos.size() > 0){
        max_y_dist = *min_element(y_dists_pos.begin(), y_dists_pos.end());
        y_dist_pos_dif = ((max_dist - max_y_dist)/(max_dist - min_dist)) * dist_max_reduction;
    }

    if (y_dists_neg.size() > 0){
        min_y_dist = *max_element(y_dists_neg.begin(), y_dists_neg.end());
        y_dist_neg_dif = ((max_dist - std::abs(min_y_dist))/(max_dist - min_dist)) * dist_max_reduction;
    }


    if (z_dists_pos.size() > 0){
        max_z_dist = *min_element(z_dists_pos.begin(), z_dists_pos.end());
        z_dist_pos_dif = ((max_dist - max_z_dist)/(max_dist - min_dist)) * dist_max_reduction;
    }

    if (z_dists_neg.size() > 0){
        min_z_dist = *max_element(z_dists_neg.begin(), z_dists_neg.end());
        z_dist_neg_dif = ((max_dist - std::abs(min_z_dist))/(max_dist - min_dist)) * dist_max_reduction;
    }
}
Eigen::VectorXd HIROAvoidance::computeJointVelocitiesWithExtCartForce(Eigen::VectorXd& q, Eigen::Vector3d xDot,
                                                    Eigen::Vector3d & externalCartesianForce,
                                                    Eigen::Vector3d & externalWrench,
                                                    std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                    std::vector<KDLSolver::closest_point>& closestPoints,
                                                    rclcpp::Rate& r, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_goal,
                                                    Eigen::Vector3d EEVelocity,
                                                    bool& inContact)
{


    resetComputationObjects();
    // std::cout << "x: " << xDot(0) << " y: " << xDot(1) << " z: " << xDot(2) << std::endl;

    Eigen::VectorXd lower_bound{_jointLimits.jointLimitsMax.size()}, upper_bound{_jointLimits.jointLimitsMax.size()};
    getVelocityLimits(lower_bound, upper_bound, q, r);

    alterEEVelocityFromObstacles(xDot, obstaclePositionVectors, ee_pos);

    // std::cout << "dx: " << xDot(0) << " dy: " << xDot(1) << " dz: " << xDot(2) << std::endl;


    updateExistingObstacleEEForce(xDot);

    computeObstacleThresholdReduction();

    updateExistingObstacleThresholdReduction();

    checkForContact(externalWrench);

    // std::cout << "cx: " << xDot(0) << " cy: " << xDot(1) << " cz: " << xDot(2) << std::endl;


    applyContactBehavior(externalWrench);

    // std::cout << "bx: " << xDot(0) << " by: " << xDot(1) << " bz: " << xDot(2) << std::endl;


    updateExistingContactForce(xDot, inContact);

    // std::cout << "ax: " << xDot(0) << " ay: " << xDot(1) << " az: " << xDot(2) << std::endl;


    // construct h matrix and f vector for qp formulation
    RCLCPP_INFO(rclcpp::get_logger("HIROAvoidance"), "Constructing H matrix and f vector");
    Eigen::MatrixXd H = getHMatrix(q, xDot, drop_goal);
    Eigen::VectorXd f = getfVector(q, xDot, drop_goal);

    Eigen::MatrixXd A, newA;
    Eigen::VectorXd b, newb;

    // get jacobian to use for after admittance control
    // Eigen::MatrixXd J = kdlSolver.computeJacobian(std::string("panda_EE"), q);
    // J = J.block(0,0,3,7);

    Eigen::VectorXd x_dot_external = cartesianComplianceMatrix * externalCartesianForce;

    int m = obstaclePositionVectors.size(); //+ objectHistory.size();
    forceBasedRepulsiveVelocity = Eigen::Vector3d(0, 0, 0);
    A = Eigen::MatrixXd(m, 7);
    b = Eigen::VectorXd(m);

    newA = Eigen::MatrixXd(1, 7);
    newb = Eigen::VectorXd(7);
    Eigen::MatrixXd Ji = Eigen::MatrixXd::Zero(3, 7);
    Eigen::MatrixXd JiResized = Eigen::MatrixXd::Zero(3, 7);

    // algorithm below:

    // 1) select the control point closest to each obstacle
    // 2) Calculate the distance d from the obstacle point to chosen control point
    // 3) Get Jacobian to selected control point. This is changing is size dependent on which control point we've selected
    // 4) Alter the size of the Jacobian so that it's always 7x3
    // comment (from matt): what and where is equation 5? and fig 5?
    // 5) Add extra row from equation 5
    // 6) d norm calculation done in Fig. 5
    // 7) Take the gradient of the norm distance with
    int numberOfRestrictions = 0;
    // for (int i = 0; i < objectHistory.size(); i++){
    //     JiResized = Eigen::MatrixXd::Zero(3, 7);
    //     b[i] = computebvalue(std::get<1>(objectHistory[i]).distance_to_obs); // From fig. 5
    //     if (!std::isnan(b(i))){
    //         numberOfRestrictions++;
    //     }
    //     Ji = kdlSolver.computeJacobian2(std::get<1>(objectHistory[i]), q);
    //     JiResized.block(0, 0, 3, Ji.cols()) = Ji.block(0, 0, 3, Ji.cols());
    //     A.row(i) = (std::get<0>(objectHistory[i]) - std::get<1>(objectHistory[i]).control_point).normalized().transpose() * JiResized;
    // }


    for (int i = objectHistory.size(); i < m; i++){
        JiResized = Eigen::MatrixXd::Zero(3, 7);
        b[i] = computebvalue(closestPoints[i].distance_to_obs); // From fig. 5
        if (!std::isnan(b(i))){
            numberOfRestrictions++;
            // objectHistory.push_back(std::make_tuple(obstaclePositionVectors[i], closestPoints[i]));
        }
        Ji = kdlSolver.computeJacobian2(closestPoints[i], q);
        JiResized.block(0, 0, 3, Ji.cols()) = Ji.block(0, 0, 3, Ji.cols());
        // A.row(i) = (obstaclePositionVectors[i] - closestPoints[i].control_point).normalized().transpose() * JiResized;
        A.row(i) = (obstaclePositionVectors[i] - closestPoints[i].control_point).normalized().transpose() * JiResized;
    }
    // constraints formulation

    newA.resize(numberOfRestrictions, A.cols());
    newb.resize(numberOfRestrictions);

    int j = 0;
    // loop through amt of obstacles
    for (int i = 0; i < m; i++){
        if (!std::isnan(b(i))){
            // if the obstacle is close, update the new A matrix
            newA.row(j) = A.row(i);
            // update the new b vector
            newb(j) = b(i);
            // increment for the A matrix, b vectors
            j++;
        }
    }
    return algLib(H, f, newA, newb, lower_bound, upper_bound);
}


/*
    gets the gradient of the distance norm, used for hiro avoidance
*/
Eigen::VectorXd HIROAvoidance::gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, KDLSolver::closest_point closestPoint, Eigen::VectorXd q)
{
    // The value of the derivative is dependent on h, generally the smaller, the better the approximation.
    Eigen::VectorXd qplus(7), qminus(7), result(7);
    double h{0.001};
    for (int i = 0; i < 7; i++)
    {
        qplus = q;
        qminus = q;
        qplus[i] = qplus[i] + h;
        qminus[i] = qminus[i] - h;
        // taylor series approximation of the derivative
        // in short, the below line(s) calculate (approximate)
        // how fast the vector from the control point to the obstacle is changing.
        result[i] = ((obstaclePositionVector - kdlSolver.forwardKinematics(closestPoint, qplus)).norm() -
             (obstaclePositionVector - kdlSolver.forwardKinematics(closestPoint, qminus)).norm()) / (2*h);
    }
    return result;
}

/*
    compute b value, given the distance norm
    intuitively, the returned value will be smaller
    if we are closer to an obstacle.
*/
double HIROAvoidance::computebvalue(double distanceNorm)
{

    // return NAN;
    // equation #13 from flacco's paper

    //Initial Values
    //changed 12/7/2021 for demo
    // double dcritical{0.2}, dnoticeable{0.6}, VrepulsiveMax{0.04};
    // double dcritical{0.1}, dnoticeable{0.4}, VrepulsiveMax{0.035};
    double dcritical{0.15}, dnoticeable{0.4}, VrepulsiveMax{0.1};

    if (distanceNorm < dnoticeable)
    {
        double result;
        // if distance is very low from closest control point
        // to obs
        if (distanceNorm < dcritical)
        {
            result = VrepulsiveMax / (1 + std::exp(-10*(2*distanceNorm/dcritical - 1))) - VrepulsiveMax;
        }
        else
        {
            result = VrepulsiveMax / (1 + std::exp(-10*(2*(distanceNorm-dcritical)/(dnoticeable- dcritical) - 1)));
        }
        return result;
    }
    else
    {
        return NAN;
    }

}

