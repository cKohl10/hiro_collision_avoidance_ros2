#include "CollisionAvoidanceBase.h"

/*
    run quadratic programming.
    while it is used with 
*/
Eigen::VectorXd CollisionAvoidanceBase::algLib(Eigen::MatrixXd H, Eigen::VectorXd f, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd bl, Eigen::VectorXd bu){
    bool successfulOptimization = false;
    bool numberOfAttemptsExceeded = false;
    while (!successfulOptimization)
    {
        try
        {
            ALGLIBAb = "[[]]";
            ALGLIBconstraintType = "[]";

            for (int i = 0; i < H.rows(); i++)
            {
                for (int j = 0; j < H.cols(); j++)
                {
                    ALGLIBH(i,j) = H(i,j);

                }
                ALGLIBf(i) = f(i);
                ALGLIBbl(i) = bl(i);
                ALGLIBbu(i) = bu(i);
            }

            if (A.rows() == 0)
            {
                ALGLIBAb = "[[]]";
            }
            else
            {
                ALGLIBAb.setlength(A.rows(), A.cols() + 1);
                ALGLIBconstraintType.setlength(A.rows());
                for (int i = 0; i < A.rows(); i++)
                {
                    for (int j = 0; j < A.cols(); j++)
                    {
                        ALGLIBAb(i,j) = A(i,j);
                    }
                    ALGLIBAb(i, A.cols()) = b(i);
                    ALGLIBconstraintType(i) = -1;
                }
            }

            // create solver, set quadratic/linear terms
            alglib::minqpcreate(ALGLIBH.cols(), ALGLIBstate);
            alglib::minqpsetquadraticterm(ALGLIBstate, ALGLIBH);
            alglib::minqpsetlinearterm(ALGLIBstate, ALGLIBf);
            alglib::minqpsetlc(ALGLIBstate, ALGLIBAb, ALGLIBconstraintType);
            alglib::minqpsetbc(ALGLIBstate, ALGLIBbl, ALGLIBbu);

            // Set scale of the parameters.
            // It is strongly recommended that you set scale of your variables.
            // Knowing their scales is essential for evaluation of stopping criteria
            // and for preconditioning of the algorithm steps.
            // You can find more information on scaling at http://www.alglib.net/optimization/scaling.php
            //
            // NOTE: for convex problems you may try using minqpsetscaleautodiag()
            //       which automatically determines variable scales.
            alglib::minqpsetscale(ALGLIBstate, ALGLIBscale);
            // alglib::minqpsetscaleautodiag(ALGLIBstate);


            // SOLVE: 3 options: BLEIC-based, DENSE-AUL, QUICKQP

            // BLEIC-based QP solver is intended for problems with moderate (up to 50) number
            // of general linear constraints and unlimited number of box constraints.
            //
            // Default stopping criteria are used.
            //
            alglib::minqpsetalgobleic(ALGLIBstate, 0.0, 0.0, 0.0, 70);
            alglib::minqpoptimize(ALGLIBstate);
            alglib::minqpresults(ALGLIBstate, ALGLIBqDot, ALGLIBrep);
            if (ALGLIBrep.terminationtype < 0)
            {
                if (numberOfAttemptsExceeded)
                {
                    RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Error in the optimization process");
                    RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Optimization exit code: %s", std::to_string(ALGLIBrep.terminationtype).c_str());
                    RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Number of attemps exceeded");
                    RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Returning zero velocities");
                    qDot = Eigen::VectorXd::Zero(7);
                    return qDot;
                }
                else
                {
                    successfulOptimization = false;
                    numberOfAttemptsExceeded = true;
                    std::cout << "-------------------------------" << std::endl;
                    RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Error in the optimization process");
                    RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Optimization exit code: %s", std::to_string(ALGLIBrep.terminationtype).c_str());
                    RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Trying easier constraints. Repulsive actions set to 0.");
                    for (int i = 0; i < b.size(); i++)
                    {
                        if (b(i) < 0)
                        {
                            b(i) = 0;
                        }
                    }
                }
            }
            else
            {
                successfulOptimization = true;
                for (int i = 0; i < ALGLIBqDot.length(); i++)
                {
                    qDot(i) = ALGLIBqDot(i);
                }
            }

        }
        catch(alglib::ap_error e)
        {
            RCLCPP_WARN(rclcpp::get_logger("CollisionAvoidanceBase"), "Alglib exception. Shutting down...");
            printf("error msg: %s\n", e.msg.c_str());
            printf("A: \n");
            std::cout << A << std::endl;
            printf("b: \n");
            std::cout << b << std::endl;
            rclcpp::shutdown();
        }
    }
    return qDot;
}


CollisionAvoidanceBase::CollisionAvoidanceBase(){
    ALGLIBH.setlength(7, 7);
    ALGLIBf.setlength(7);
    ALGLIBbl.setlength(7);
    ALGLIBbu.setlength(7);
}
