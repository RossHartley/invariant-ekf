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


// // Move initialization
// RobotState::RobotState(RobotState&& other) {
//     lock_guard<mutex> lock(other.mutex_);
//     X_ = std::move(other.X_);
//     other.X_ = Eigen::MatrixXd;
// }

#if INEKF_USE_MUTEX
// Copy initialization
RobotState::RobotState(const RobotState& other) {
    lock_guard<mutex> other_lock(other.mutex_);
    X_ = other.X_;
    Theta_ = other.Theta_;
    P_ = other.P_;
}

// // Move assignment
// RobotState::RobotState& operator = (RobotState&& other) {
//     std::lock(mtx, other.mtx);
//     std::lock_guard<std::mutex> self_lock(mtx, std::adopt_lock);
//     std::lock_guard<std::mutex> other_lock(other.mtx, std::adopt_lock);
//     value = std::move(other.value);
//     other.value = 0;
//     return *this;
// }

// Copy assignment
RobotState& RobotState::operator = (const RobotState& other) {
    lock(mutex_, other.mutex_);
    lock_guard<mutex> self_lock(mutex_, adopt_lock);
    lock_guard<mutex> other_lock(other.mutex_, adopt_lock);
    X_ = other.X_;
    Theta_ = other.Theta_;
    P_ = other.P_;
    return *this;
}
#endif


const Eigen::MatrixXd RobotState::getX() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_; 
}
const Eigen::VectorXd RobotState::getTheta() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_; 
}
const Eigen::MatrixXd RobotState::getP() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return P_; 
}
const Eigen::Matrix3d RobotState::getRotation() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,3>(0,0); 
}
const Eigen::Vector3d RobotState::getVelocity() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,1>(0,3); 
}
const Eigen::Vector3d RobotState::getPosition() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,1>(0,4); 
}
const Eigen::Vector3d RobotState::getGyroscopeBias() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.head(3); 
}
const Eigen::Vector3d RobotState::getAccelerometerBias() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.tail(3); 
}
const int RobotState::dimX() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.cols(); 
}
const int RobotState::dimTheta() {
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.rows();
}
const int RobotState::dimP() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return P_.cols(); 
}

void RobotState::setX(const Eigen::MatrixXd& X) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_ = X; 
}
void RobotState::setTheta(const Eigen::VectorXd& Theta) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_ = Theta; 
}
void RobotState::setP(const Eigen::MatrixXd& P) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    P_ = P; 
}
void RobotState::setRotation(const Eigen::Matrix3d& R) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,3>(0,0) = R; 
}
void RobotState::setVelocity(const Eigen::Vector3d& v) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,1>(0,3) = v; 
}
void RobotState::setPosition(const Eigen::Vector3d& p) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,1>(0,4) = p; 
}
void RobotState::setGyroscopeBias(const Eigen::Vector3d& bg) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_.head(3) = bg; 
}
void RobotState::setAccelerometerBias(const Eigen::Vector3d& ba) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_.tail(3) = ba; 
}


void RobotState::copyDiagX(int n, Eigen::MatrixXd& BigX) {
    int dimX = this->dimX();
    for(int i=0; i<n; ++i) {
        int startIndex = BigX.rows();
        BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigX.block(startIndex,0,dimX,startIndex) = Eigen::MatrixXd::Zero(dimX,startIndex);
        BigX.block(0,startIndex,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
#if INEKF_USE_MUTEX
        unique_lock<mutex> mlock(mutex_);
#endif
        BigX.block(startIndex,startIndex,dimX,dimX) = X_;
    }
    return;
}

ostream& operator<<(ostream& os, const RobotState& s) {  
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(s.mutex_);
#endif
    os << "--------- Robot State -------------" << endl;
    os << "X:\n" << s.X_ << endl << endl;
    os << "Theta:\n" << s.Theta_ << endl << endl;
    os << "P:\n" << s.P_ << endl;
    os << "-----------------------------------";
    return os;  
} 

} // end inekf namespace
