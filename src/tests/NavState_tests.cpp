/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NavState_tests.cpp
 *  @author Ross Hartley
 *  @brief  
 *  @date   May 23, 2019
 **/

#include <iostream>
#include "InEKF.h"

int main() {
    
    // Default constructor
    inekf::NavState state1;
    assert(state1.getRotation() == Eigen::Matrix3d::Identity());
    assert(state1.getVelocity() == Eigen::Vector3d::Zero());
    assert(state1.getPosition() == Eigen::Vector3d::Zero());
    assert(state1.getGyroscopeBias() == Eigen::Vector3d::Zero());
    assert(state1.getAccelerometerBias() == Eigen::Vector3d::Zero());
    std::cout << "Passed default constructor test.\n";

    // Constructor from state
    Eigen::Vector3d phi; phi << 1,2,3;
    Eigen::Matrix3d R = inekf::Exp_SO3(phi);
    Eigen::Vector3d v; v << 4,5,6;
    Eigen::Vector3d p; p << 7,8,9;
    Eigen::Vector3d bg; bg << 10,11,12;
    Eigen::Vector3d ba; ba << 13,14,15;
    inekf::NavState state2(R,v,p,bg,ba);
    assert(state2.getRotation() == R);
    assert(state2.getVelocity() == v);
    assert(state2.getPosition() == p);
    assert(state2.getGyroscopeBias() == bg);
    assert(state2.getAccelerometerBias() == ba);
    std::cout << "Passed state initialization constructor test.\n";

    // Test inverse
    inekf::NavState state3 = state2.inverse();
    assert(state3.getRotation() == state2.getRotation().transpose());
    assert(state3.getVelocity() == (-state2.getRotation().transpose()*state2.getVelocity()).eval());
    assert(state3.getPosition() == (-state2.getRotation().transpose()*state2.getPosition()).eval()); 
    std::cout << "Passed inverse test.\n";

    // Test Integration
    Eigen::Vector3d w; w << 1,2,3;
    Eigen::Vector3d a; a << 4,5,6;
    double dt = 0.1;
    Eigen::Matrix<double,15,15> Phi, Qd;
    Eigen::Matrix<double,15,15> Adj_XkInv = Eigen::Matrix<double,15,15>::Identity();
    Adj_XkInv.block<9,9>(0,0) = inekf::Adjoint_SEK3(state3.inverse().getX());
    state3.integrate(w,a,dt,Phi); // Integrate and compute state transition matrix (right-invariant)
    Eigen::Matrix<double,15,15> Adj_Xk1 = Eigen::Matrix<double,15,15>::Identity();
    Adj_Xk1.block<9,9>(0,0) = inekf::Adjoint_SEK3(state3.getX());

    // Compute state transition matrix using matrix exponential (left-invariant)
    Eigen::Matrix<double,15,15> A = Eigen::Matrix<double,15,15>::Zero();
    Eigen::Matrix3d wx = inekf::skew(w-bg);
    Eigen::Matrix3d ax = inekf::skew(a-ba);
    A.block<3,3>(0,0) = -wx;  
    A.block<3,3>(3,3) = -wx;  
    A.block<3,3>(6,6) = -wx;  
    A.block<3,3>(3,0) = -ax; 
    A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();  
    A.block<3,3>(0,9) = -Eigen::Matrix3d::Identity();  
    A.block<3,3>(3,12) = -Eigen::Matrix3d::Identity();  
    Eigen::Matrix<double,15,15> Phi_L = (A*dt).exp();
    Eigen::Matrix<double,15,15> Phi_R = Adj_Xk1 * Phi_L * Adj_XkInv;
    assert((Phi-Phi_R).norm()<1e-10);
    std::cout << "Passed integration and state transition matrix test.\n";

    return 0;
}
