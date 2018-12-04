/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Source file for RobotState (thread-safe)
 *  @date   September 25, 2018
 **/

#include "RobotState.h"
#include "LieGroup.h"

namespace inekf {

using namespace std;

// Default constructor
RobotState::RobotState() : 
    X_(Eigen::MatrixXd::Identity(5,5)), Theta_(Eigen::MatrixXd::Zero(6,1)), P_(Eigen::MatrixXd::Identity(15,15)) {}
// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd& X) : 
    X_(X), Theta_(Eigen::MatrixXd::Zero(6,1)) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta) : 
    X_(X), Theta_(Theta) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P) : 
    X_(X), Theta_(Theta), P_(P) {}
// TODO: error checking to make sure dimensions are correct and supported

const Eigen::MatrixXd RobotState::getX() const { return X_; }
const Eigen::VectorXd RobotState::getTheta() const { return Theta_; }
const Eigen::MatrixXd RobotState::getP() const { return P_; }
const Eigen::Matrix3d RobotState::getRotation() const { return X_.block<3,3>(0,0); }
const Eigen::Vector3d RobotState::getVelocity() const { return X_.block<3,1>(0,3); }
const Eigen::Vector3d RobotState::getPosition() const { return X_.block<3,1>(0,4); }
const Eigen::Vector3d RobotState::getGyroscopeBias() const { return Theta_.head(3); }
const Eigen::Vector3d RobotState::getAccelerometerBias() const { return Theta_.tail(3); }

const Eigen::Matrix3d RobotState::getRotationCovariance() const { return P_.block<3,3>(0,0); }
const Eigen::Matrix3d RobotState::getVelocityCovariance() const { return P_.block<3,3>(3,3); }
const Eigen::Matrix3d RobotState::getPositionCovariance() const { return P_.block<3,3>(6,6); }
const Eigen::Matrix3d RobotState::getGyroscopeBiasCovariance() const { return P_.block<3,3>(P_.rows()-6,P_.rows()-6); }
const Eigen::Matrix3d RobotState::getAccelerometerBiasCovariance() const { return P_.block<3,3>(P_.rows()-3,P_.rows()-3); }

const int RobotState::dimX() const { return X_.cols(); }
const int RobotState::dimTheta() const {return Theta_.rows();}
const int RobotState::dimP() const { return P_.cols(); }

void RobotState::setX(const Eigen::MatrixXd& X) { X_ = X; }
void RobotState::setTheta(const Eigen::VectorXd& Theta) { Theta_ = Theta; }
void RobotState::setP(const Eigen::MatrixXd& P) { P_ = P; }
void RobotState::setRotation(const Eigen::Matrix3d& R) { X_.block<3,3>(0,0) = R; }
void RobotState::setVelocity(const Eigen::Vector3d& v) { X_.block<3,1>(0,3) = v; }
void RobotState::setPosition(const Eigen::Vector3d& p) { X_.block<3,1>(0,4) = p; }
void RobotState::setGyroscopeBias(const Eigen::Vector3d& bg) { Theta_.head(3) = bg; }
void RobotState::setAccelerometerBias(const Eigen::Vector3d& ba) { Theta_.tail(3) = ba; }
void RobotState::setRotationCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(0,0) = cov; }
void RobotState::setVelocityCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(3,3) = cov; }
void RobotState::setPositionCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(6,6) = cov; }
void RobotState::setGyroscopeBiasCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(P_.rows()-6,P_.rows()-6) = cov; }
void RobotState::setAccelerometerBiasCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(P_.rows()-3,P_.rows()-3) = cov; }

void RobotState::copyDiagX(int n, Eigen::MatrixXd& BigX) const {
    int dimX = this->dimX();
    for(int i=0; i<n; ++i) {
        int startIndex = BigX.rows();
        BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigX.block(startIndex,0,dimX,startIndex) = Eigen::MatrixXd::Zero(dimX,startIndex);
        BigX.block(0,startIndex,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
        BigX.block(startIndex,startIndex,dimX,dimX) = X_;
    }
    return;
}

void RobotState::copyDiagXinv(int n, Eigen::MatrixXd& BigXinv) const {
    int dimX = this->dimX();
    Eigen::MatrixXd Xinv;
    Xinv.resize(dimX,dimX);
    Xinv = X_.inverse(); // TODO: Replace with simple solution
    for(int i=0; i<n; ++i) {
        int startIndex = BigXinv.rows();
        BigXinv.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigXinv.block(startIndex,0,dimX,startIndex) = Eigen::MatrixXd::Zero(dimX,startIndex);
        BigXinv.block(0,startIndex,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);

        BigXinv.block(startIndex,startIndex,dimX,dimX) = Xinv;
    }
    return;
}

ostream& operator<<(ostream& os, const RobotState& s) {  
    os << "--------- Robot State -------------" << endl;
    os << "X:\n" << s.X_ << endl << endl;
    os << "Theta:\n" << s.Theta_ << endl << endl;
    // os << "P:\n" << s.P_ << endl;
    os << "-----------------------------------";
    return os;  
} 

} // end inekf namespace
