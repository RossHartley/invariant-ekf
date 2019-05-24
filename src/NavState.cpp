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
    bg_(Eigen::Vector3d::Zero()), ba_(Eigen::Vector3d::Zero()), g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()) {}

// Initialize with provided state
NavState::NavState(const Eigen::Matrix3d& R, const Eigen::Vector3d& v, const Eigen::Vector3d& p, 
                   const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) :
    R_(R), v_(v), p_(p), bg_(bg), ba_(ba), g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()) {}


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

// Integrate IMU data
void NavState::integrate(const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& linear_acceleration, double dt, 
                         boost::optional<Eigen::Matrix<double,15,15>&> Phi, std::string error_type) {
    // Bias corrected IMU measurements
    Eigen::Vector3d w = angular_velocity - bg_;
    Eigen::Vector3d a = linear_acceleration - ba_;

    // Auxiliary quantities
    double dt2 = dt*dt;
    Eigen::Vector3d phi = w*dt;
    Eigen::Matrix3d G0 = Gamma_SO3(phi,0); // Computation can be sped up by computing G0,G1,G2 all at once
    Eigen::Matrix3d G1 = Gamma_SO3(phi,1);
    Eigen::Matrix3d G2 = Gamma_SO3(phi,2);

    // Propagate world-centric state estimate (order matters!)
    p_ = (p_ + v_*dt + (R_*G2*a + 0.5*g_)*dt2).eval();
    v_ = (v_ + (R_*G1*a + g_)*dt).eval();
    R_ = (R_ * G0).eval();

    // Optional state transition matrix
    if (Phi) {
        // Helper quantities for bias terms
        Eigen::Matrix3d wx = skew(w);
        Eigen::Matrix3d ax = skew(a);
        Eigen::Vector3d phi = w*dt;
        Eigen::Matrix3d phix = skew(phi);
        Eigen::Matrix3d phixax = phix*ax;
        Eigen::Matrix3d phix2 = phix*phix;
        Eigen::Matrix3d phix2ax = phix2*ax;
        double theta = phi.norm();
        double theta2 = theta*theta;
        double theta3 = theta2*theta;
        double theta4 = theta3*theta;
        double theta5 = theta4*theta;
        double theta6 = theta5*theta;
        double theta7 = theta6*theta;
        double sintheta = sin(theta);
        double costheta = cos(theta);
        double sin2theta = sin(2*theta);
        double cos2theta = cos(2*theta);

        Eigen::Matrix3d Psi1 = ax*Gamma_SO3(-phi,2)
            + ((sintheta-theta*costheta)/(theta3))*(phixax)
            - ((cos2theta-4*costheta+3)/(4*theta4))*(phixax*phix)
            + ((4*sintheta+sin2theta-4*theta*costheta-2*theta)/(4*theta5))*(phixax*phix2)
            + ((theta2-2*theta*sintheta-2*costheta+2)/(2*theta4))*(phix2ax)
            - ((6*theta-8*sintheta+sin2theta)/(4*theta5))*(phix2ax*phix)
            + ((2*theta2-4*theta*sintheta-cos2theta+1)/(4*theta6))*(phix2ax*phix2);

        Eigen::Matrix3d Psi2 = ax*Gamma_SO3(-phi,3) 
            - ((theta*sintheta+2*costheta-2)/(theta4))*(phixax) 
            - ((6*theta-8*sintheta+sin2theta)/(8*theta5))*(phixax*phix) 
            - ((2*theta2+8*theta*sintheta+16*costheta+cos2theta-17)/(8*theta6))*(phixax*phix2) 
            + ((theta3+6*theta-12*sintheta+6*theta*costheta)/(6*theta5))*(phix2ax) 
            - ((6*theta2+16*costheta-cos2theta-15)/(8*theta6))*(phix2ax*phix) 
            + ((4*theta3+6*theta-24*sintheta-3*sin2theta+24*theta*costheta)/(24*theta7))*(phix2ax*phix2);

        // State transition matrix depends on error definition
        if (error_type.compare("left")==0) { 
            // left-invariant error
            (*Phi) = Eigen::Matrix<double,15,15>::Identity();
            Eigen::Matrix3d G0t = G0.transpose(); 
            (*Phi).block<3,3>(0,0) = G0t;
            (*Phi).block<3,3>(3,0) = -G0t*skew(G1*a)*dt;
            (*Phi).block<3,3>(6,0) = -G0t*skew(G2*a)*dt2;
            (*Phi).block<3,3>(3,3) = G0t;
            (*Phi).block<3,3>(6,3) = G0t*dt;
            (*Phi).block<3,3>(6,6) = G0t;
            (*Phi).block<3,3>(0,9) = -G0t*G1*dt;
            (*Phi).block<3,3>(3,9) = G0t*Psi1*dt2;
            (*Phi).block<3,3>(6,9) = G0t*Psi2*dt2*dt;
            (*Phi).block<3,3>(3,12) = -G0t*G1*dt;
            (*Phi).block<3,3>(6,12) = -G0t*G2*dt2;
        } else if (error_type.compare("right")==0) { 
            // right-invariant error
            (*Phi) = Eigen::Matrix<double,15,15>::Identity();
            Eigen::Matrix3d gx = skew(g_);
            Eigen::Matrix3d Rk = R_*G0.transpose(); 
            Eigen::Matrix3d RkG1 = Rk*G1; 
            (*Phi).block<3,3>(3,0) = gx*dt;
            (*Phi).block<3,3>(6,0) = 0.5*gx*dt2;
            (*Phi).block<3,3>(6,3) = Eigen::Matrix3d::Identity()*dt;
            (*Phi).block<3,3>(0,9) = -RkG1*dt;
            (*Phi).block<3,3>(3,9) = -skew(v_)*RkG1*dt + Rk*Psi1*dt2;
            (*Phi).block<3,3>(6,9) = -skew(p_)*RkG1*dt + Rk*Psi2*dt2*dt;
            (*Phi).block<3,3>(3,12) = -RkG1*dt;
            (*Phi).block<3,3>(6,12) = -Rk*G2*dt2;
        } else {
            std::string error_msg = "Uknown error type (" + error_type + ") as input to NavState integrate function. Argument should be (left) or (right).";
            throw runtime_error(error_msg);
        }
    }
        
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


