/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   left_vs_right_error_dynamics.cpp
 *  @author Ross Hartley
 *  @brief  Test to make sure the left and right error dynamics are identical
 *  @date   February 18, 2019
 **/

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "InEKF.h"

using namespace std;
using namespace inekf;

int main() {
    //  ---- Initialize invariant extended Kalman filter ----- //
    RobotState initial_state; 

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0, // initial orientation
          0, -1, 0, // IMU frame is rotated 90deg about the x-axis
          0, 0, 1;
    v0 << 1,2,3; // initial velocity
    p0 << 4,5,6; // initial position
    bg0 << 1,2,3; // initial gyroscope bias
    ba0 << 1,2,3; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize noise params
    NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.0);
    noise_params.setAccelerometerNoise(0.0);
    noise_params.setGyroscopeBiasNoise(0.0);
    noise_params.setAccelerometerBiasNoise(0.0);
    noise_params.setContactNoise(0.0);

    // Initial Covariance and Adjoint
    Eigen::Matrix<double,15,15> P = Eigen::Matrix<double,15,15>::Identity();
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(initial_state.dimP(),initial_state.dimP());
    Adj.block(0,0,initial_state.dimP()-initial_state.dimTheta(),initial_state.dimP()-initial_state.dimTheta()) = Adjoint_SEK3(initial_state.getX()); 
    
    // Left invariant filter
    initial_state.setP(P);
    InEKF LI_filter(initial_state, noise_params, ErrorType::LeftInvariant);

    // Right invariant filter
    initial_state.setP(Adj*P*Adj.transpose());
    InEKF RI_filter(initial_state, noise_params, ErrorType::RightInvariant);

    // Propagate both
    Eigen::Matrix<double,6,1> imu;
    imu << 1,2,3,4,5,6;
    double dt = 0.0005;
    for (int i=0; i<2000; ++i) {
        LI_filter.Propagate(imu, dt); 
        RI_filter.Propagate(imu, dt); 
    }
    // Print covariances
    RobotState LI_state = LI_filter.getState();
    RobotState RI_state = RI_filter.getState();
    cout << "Left Invariant State: \n" << LI_state << endl;
    cout << "Right Invariant State: \n" << RI_state << endl;

    cout << "Left Invariant Covariance: \n" << LI_state.getP() << endl << endl;
    cout << "Right Invariant Covariance: \n" << RI_state.getP() << endl << endl;

    Adj.block(0,0,LI_state.dimP()-LI_state.dimTheta(),LI_state.dimP()-LI_state.dimTheta()) = Adjoint_SEK3(LI_state.getX()); 
    cout << "Difference between right invariant covariance (left is mapped using adjoint): \n" << (RI_state.getP() - (Adj * LI_state.getP() * Adj.transpose()).eval()).norm() << endl << endl;

    Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(RI_state.dimP(),RI_state.dimP());
    AdjInv.block(0,0,RI_state.dimP()-RI_state.dimTheta(),RI_state.dimP()-RI_state.dimTheta()) = Adjoint_SEK3(RI_state.getX().inverse()); 
    cout << "Difference between left invariant covariance (right is mapped using adjoint inverse): \n" << (LI_state.getP() - (AdjInv * RI_state.getP() * AdjInv.transpose()).eval()).norm() << endl << endl;

    return 0;
}
