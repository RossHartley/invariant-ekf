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
    
    return 0;
}
