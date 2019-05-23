/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NavState.h
 *  @author Ross Hartley
 *  @brief  Source file for NavState
 *  @date   May 23, 2019
 **/

#include "NavState.h"
#include "LieGroup.h"

namespace inekf {

using namespace std;

// Default constructor
NavState::NavState() : 
    R_(Eigen::Matrix3d::Identity()), v_(Eigen::Vector3d::Zero()), p_(Eigen::Vector3d::Zero()), 
    bg_(Eigen::Vector3d::Zero()), ba_(Eigen::Vector3d::Zero()) {}

// Initialize with provided state
NavState::NavState(const Eigen::Matrix3d& R, const Eigen::Vector3d& v, const Eigen::Vector3d& p, 
                   const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) :
    R_(R), v_(v), p_(p), bg_(bg), ba_(ba) {}


// Get separate state variables
const Eigen::Matrix3d NavState::getRotation() const { return R_; }
const Eigen::Vector3d NavState::getVelocity() const { return v_; }
const Eigen::Vector3d NavState::getPosition() const { return p_; }
const Eigen::Vector3d NavState::getGyroscopeBias() const { return bg_; }
const Eigen::Vector3d NavState::getAccelerometerBias() const { return ba_; }

// Get full state Lie Group
const Eigen::Matrix<double,5,5> NavState::getX() const { 
    Eigen::Matrix<double,5,5> X = Eigen::Matrix<double,5,5>::Identity();
    X.block<3,3>(0,0) = R_;
    X.block<3,1>(0,3) = v_;
    X.block<3,1>(0,4) = p_;
    return X; 
}

// Get full parameter vector
const Eigen::Matrix<double,6,1> NavState::getBias() const { 
    Eigen::Matrix<double,6,1> bias;
    bias.head(3) = bg_;
    bias.tail(3) = ba_;
    return bias;
}

// Get state dimensions
const int NavState::dimX() const { return dimX_; }
const int NavState::dimBias() const { return dimBias_; }

// Set separate state variables
void NavState::setRotation(const Eigen::Matrix3d& R) { R_ = R; }
void NavState::setVelocity(const Eigen::Vector3d& v) { v_ = v; }
void NavState::setPosition(const Eigen::Vector3d& p) { p_ = p; }
void NavState::setGyroscopeBias(const Eigen::Vector3d& bg) { bg_ = bg; }
void NavState::setAccelerometerBias(const Eigen::Vector3d& ba) { ba_ = ba; }

// Set full state Lie Group
const Eigen::Matrix<double,5,5> NavState::setX(const Eigen::Matrix<double,5,5>& X) { 
    R_ = X.block<3,3>(0,0);
    v_ = X.block<3,1>(0,3);
    p_ = X.block<3,1>(0,4);
    return X; 
}

// Set full parameter vector
const Eigen::Matrix<double,6,1> NavState::setBias(const Eigen::Matrix<double,6,1>& bias) { 
    bg_ = bias.head(3);
    ba_ = bias.tail(3);
    return bias;
}

// Inverse
NavState NavState::inverse() const {
    Eigen::Matrix3d Rt = R_.transpose();
    return NavState(Rt, -Rt*v_, -Rt*p_, bg_, ba_);
}

// Print
ostream& operator<<(ostream& os, const NavState& s) {  
    os << "--------- NavState -------------" << endl;
    os << "X:\n" << s.getX() << endl << endl;
    os << "bias: " << s.getBias().transpose() << endl;
    os << "-----------------------------------\n";
    return os;  
} 

} // end inekf namespace
