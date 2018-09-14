#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H 
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace inekf {

class RobotState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RobotState();
        RobotState(const Eigen::MatrixXd& X);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P);
        
        // RobotState(RobotState&& other); // Move initialization
        RobotState(const RobotState& other); // Copy initialization
        // RobotState& operator=(RobotState&& other); // Move assignment
        RobotState& operator=(const RobotState& other); // Copy assignment

        const Eigen::MatrixXd getX();
        const Eigen::VectorXd getTheta();
        const Eigen::MatrixXd getP();
        const Eigen::Matrix3d getRotation();
        const Eigen::Vector3d getVelocity();
        const Eigen::Vector3d getPosition();
        const Eigen::Vector3d getAngularVelocityBias();
        const Eigen::Vector3d getLinearAccelerationBias();
        const int dimX();
        const int dimTheta();
        const int dimP();

        void setX(Eigen::MatrixXd& X);
        void setP(Eigen::MatrixXd& P);
        void setTheta(Eigen::VectorXd& Theta);
        void setRotation(Eigen::Matrix3d& R);
        void setVelocity(Eigen::Vector3d& v);
        void setPosition(Eigen::Vector3d& p);
        void setAngularVelocityBias(Eigen::Vector3d& bg);
        void setLinearAccelerationBias(Eigen::Vector3d& ba);

        void copyDiagX(int n, Eigen::MatrixXd& BigX);

        friend class InEKF;
        friend std::ostream& operator<<(std::ostream& os, const RobotState& s);  

    private:
        Eigen::MatrixXd X_;
        Eigen::VectorXd Theta_;
        Eigen::MatrixXd P_;
        mutable std::mutex mutex_;

};

} // end inekf namespace
#endif 

