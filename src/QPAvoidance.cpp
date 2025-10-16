// Link to Ding Paper Presentation:
// https://docs.google.com/presentation/d/1LrW7mna1wRgHsIzw3wXOrvIg3xlkNpIfmVRfGyxG_v0/edit?usp=sharing

// Link to Overleaf file with the math in the presentation
// https://www.overleaf.com/read/hwndqxxqtvds

#include "QPAvoidance.h"


/*
    qp avoidance -- setup for alglib
*/
QPAvoidance::QPAvoidance(){}

QPAvoidance::QPAvoidance(JointLimits jointLimits){
    _jointLimits = jointLimits;
}

int QPAvoidance::calculateOptimizationRowValues(Eigen::MatrixXd& A, Eigen::VectorXd& b,
                                                const  std::vector<Eigen::Vector3d>& obstaclePositionVectorsOriginal, 
                                                const std::vector<KDLSolver::closest_point>& closestPointsOriginal,
                                                const Eigen::VectorXd& qOriginal){

            Eigen::MatrixXd Ji = Eigen::MatrixXd::Zero(3, 7);
            Eigen::MatrixXd JiResized = Eigen::MatrixXd::Zero(3, 7);

            // TODO: Caleb - I should rethink how I pass variables to this fxn
            std::vector<KDLSolver::closest_point>closestPoints = closestPointsOriginal;
            std::vector<Eigen::Vector3d>obstaclePositionVectors = obstaclePositionVectorsOriginal;
            std::vector<Eigen::MatrixXd> controlPointsJc(closestPoints.size());
            Eigen::VectorXd q  =  qOriginal;

            int numObstacles = obstaclePositionVectors.size();
            int numberOfRestrictions = 0;

            // Note: The + 1 below accounts for 
            // the gradient restrction introduced in
            // equation 6. This restriciton is placed as 
            // the last element is both A and b below.
            Eigen::MatrixXd ATemp = Eigen::MatrixXd(numObstacles + 1, 7);
            Eigen::VectorXd bTemp = Eigen::VectorXd(numObstacles + 1);

            Eigen::VectorXd w = Eigen::VectorXd::Ones(numObstacles);
            Eigen::MatrixXd C = Eigen::MatrixXd(numObstacles, 7);
            
            for (int i = 0; i < numObstacles; i++){
                w[i] = 1 / closestPoints[i].distance_to_obs; // eq. 6
                bTemp[i] = computebvalue(closestPoints[i].distance_to_obs); // fig. 5
                if (!std::isnan(bTemp(i))) numberOfRestrictions++;


                Ji = kdlSolver.computeJacobian2(closestPoints[i], q);
                JiResized.block(0, 0, 3, Ji.cols()) = Ji.block(0, 0, 3, Ji.cols());
                controlPointsJc[i] = JiResized;
                Eigen::VectorXd distance_normalized = (obstaclePositionVectors[i] - closestPoints[i].control_point).normalized();

                ATemp.row(i) = distance_normalized.transpose() * JiResized;
                C.row(i) = gradientOfDistanceNorm(obstaclePositionVectors[i], closestPoints[i], q);

        }

        A.resize(numberOfRestrictions + 1, 7);
        b.resize(numberOfRestrictions + 1);
        for (int i = 0, j = 0; i < obstaclePositionVectors.size(); i++)
        {
            if (!std::isnan(bTemp(i)))
            {
                A.row(j) = ATemp.row(i);
                b(j) = bTemp(i);
                j++;
            }
        }
        A.row(numberOfRestrictions) = - w.transpose() * C;
        b(numberOfRestrictions) = 0;

        // Note: Logging removed - was: loggingPublisher_.publishControlPoints(...)
        // Note: Logging removed - was: loggingPublisher_.addEEasControlPoint(...)
        // Note: Logging removed - was: loggingPublisher_.publishManipulabilityAnalysis(...)   

        return numberOfRestrictions;
}

/*
    compute joint velocities given obstacles.
*/
Eigen::VectorXd QPAvoidance::computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d& xDot,
                                                    std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                    std::vector<KDLSolver::closest_point>& closestPoints,
                                                    rclcpp::Rate& r)
{
    // All computations are based on this paper: 
    // https://www.researchgate.net/publication/339676345_Collision_Avoidance_with_Proximity_Servoing_for_Redundant_Serial_Robot_Manipulators

    Eigen::VectorXd bl{_jointLimits.jointLimitsMax.size()};
    Eigen::VectorXd bu{_jointLimits.jointLimitsMax.size()};
    calculateJointVelocityBounds(bl, bu, q, r);

    // Matrices and Vectors used in Optimization
    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
  
    setupOptimizationMainTask(H, f, q, xDot);
    

    if (obstaclePositionVectors.size() == 0)
    {
        A = Eigen::MatrixXd(0,0);
        b = Eigen::VectorXd(0);
    }
    else
    {
        calculateOptimizationRowValues(A, b, obstaclePositionVectors, closestPoints, q);
    }
    return algLib(H, f, A, b, bl, bu);
}

void QPAvoidance::calculateJointVelocityBounds(Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound, const Eigen::VectorXd& q, const rclcpp::Rate& r){
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

void QPAvoidance::setupOptimizationMainTask(Eigen::MatrixXd& H, Eigen::VectorXd& f, const Eigen::VectorXd& q, const Eigen::Vector3d& xDot){
    Eigen::MatrixXd J = kdlSolver.computeJacobian(kdlSolver.getEELink(), q).block(0,0,3,7);
    Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();

    double manipulability_measure = std::sqrt((J*J.transpose()).determinant());
    H = J.transpose() * J + computeDampingFactor(manipulability_measure) * Eigen::MatrixXd::Identity(7,7);
    f = - xDot.transpose() * J;

} 

/*
    get the gradient of distance norm for an obstacle.
*/
Eigen::VectorXd QPAvoidance::gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, KDLSolver::closest_point closestPoint, Eigen::VectorXd q)
{
    // The value of the derivative is dependent on h, generally the smaller the better the aproximation.
    // TODO: Caleb - is this really the best way tot calculate the gradient here?
    Eigen::VectorXd qplus(7), qminus(7), result(7);
    double h{0.001};
    for (int i = 0; i < 7; i++)
    {
        qplus = q;
        qminus = q;
        qplus[i] = qplus[i] + h;
        qminus[i] = qminus[i] - h;
        result[i] = ((obstaclePositionVector - kdlSolver.forwardKinematics(closestPoint, qplus)).norm() -
             (obstaclePositionVector - kdlSolver.forwardKinematics(closestPoint, qminus)).norm()) / (2*h);
    }
    return result;
}

/*
    compute b value given distance norm
*/
double QPAvoidance::computebvalue(double distanceNorm)
{
    // equation #13
    // double dr{0.3}, d0l{0.35}, d0u{0.4}, da{0.5}, xdota{0.1}, xdotr{-0.4};
    // Weights given in the paper for weights through equations 16-18
    double dr{0.17}, d0l{0.22}, d0u{0.25}, da{0.3}, xdota{0.2}, xdotr{-0.2};
    if (distanceNorm < dr)
    {
        return xdotr;
    }
    else if (distanceNorm < d0l)
    {
        return xdotr + (0 - xdotr) / (d0l - dr) * (distanceNorm - dr);
    }
    else if (distanceNorm < d0u)
    {
        return 0;
    }
    else if (distanceNorm < da)
    {
        return 0 + (xdota - 0) / (da - d0u) * (distanceNorm - d0u);
    }
    else
    {
        return NAN;
    }
}

/*
    compute damping factor based on omega
*/
double QPAvoidance::computeDampingFactor(double omega)
{
    // Given from equation 15
    if (omega >= omega0)
    {
        return 0.0;
    }
    else
    {
        return dampingFactor0 * std::pow((1 - omega/omega0),2);
    }
}
