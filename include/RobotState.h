/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Header file for RobotState
 *  @date   September 25, 2018
 **/

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H 
#include <Eigen/Dense>
#include <iostream>

namespace inekf {

class RobotState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RobotState();
        RobotState(const Eigen::MatrixXd& X);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P);

        const Eigen::MatrixXd getX() const;
        const Eigen::VectorXd getTheta() const;
        const Eigen::MatrixXd getP() const;
        const Eigen::Matrix3d getRotation() const;
        const Eigen::Vector3d getVelocity() const;
        const Eigen::Vector3d getPosition() const;
        const Eigen::Vector3d getGyroscopeBias() const;
        const Eigen::Vector3d getAccelerometerBias() const;
        const Eigen::Matrix3d getRotationCovariance() const;
        const Eigen::Matrix3d getVelocityCovariance() const;
        const Eigen::Matrix3d getPositionCovariance() const;
        const Eigen::Matrix3d getGyroscopeBiasCovariance() const;
        const Eigen::Matrix3d getAccelerometerBiasCovariance() const;
        const int dimX() const;
        const int dimTheta() const;
        const int dimP() const;

        void setX(const Eigen::MatrixXd& X);
        void setP(const Eigen::MatrixXd& P);
        void setTheta(const Eigen::VectorXd& Theta);
        void setRotation(const Eigen::Matrix3d& R);
        void setVelocity(const Eigen::Vector3d& v);
        void setPosition(const Eigen::Vector3d& p);
        void setGyroscopeBias(const Eigen::Vector3d& bg);
        void setAccelerometerBias(const Eigen::Vector3d& ba);
        void setRotationCovariance(const Eigen::Matrix3d& cov);
        void setVelocityCovariance(const Eigen::Matrix3d& cov);
        void setPositionCovariance(const Eigen::Matrix3d& cov);
        void setGyroscopeBiasCovariance(const Eigen::Matrix3d& cov);
        void setAccelerometerBiasCovariance(const Eigen::Matrix3d& cov);
        void copyDiagX(int n, Eigen::MatrixXd& BigX) const;
        void copyDiagXinv(int n, Eigen::MatrixXd& BigXinv) const;

        friend std::ostream& operator<<(std::ostream& os, const RobotState& s);  

    private:
        Eigen::MatrixXd X_;
        Eigen::VectorXd Theta_;
        Eigen::MatrixXd P_;
};

} // end inekf namespace
#endif 

