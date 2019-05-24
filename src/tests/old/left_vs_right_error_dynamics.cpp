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
#include <random>
#include <chrono>
#include "InEKF.h"

using namespace std;
using namespace inekf;

int main() {
    typedef std::chrono::high_resolution_clock myclock;
    myclock::time_point beginning = myclock::now();

    //  ---- Initialize invariant extended Kalman filter ----- //
    RobotState initial_state; 

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0, // initial orientation
          0, -1, 0, // IMU frame is rotated 90deg about the x-axis
          0, 0, -1;
    v0 << 1,2,3; // initial velocity
    p0 << 4,5,6; // initial position
    bg0 << 0,0,0; // initial gyroscope bias
    ba0 << 0,0,0; // initial accelerometer bias
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

    // obtain a seed from the timer
    myclock::duration d = myclock::now() - beginning;
    unsigned seed = d.count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0,1);

    // ----- Propagate using random data ------
    cout << "\n\n ------ Propagate using random data -------\n\n";
    const int NUM_PROPAGATE = 100;
    Eigen::Matrix<double,6,1> imu;
    for (int i=0; i<NUM_PROPAGATE; ++i) {
        for (int j=0; j<6; ++j) {
            imu(j) = distribution(generator);
        }
        double dt = distribution(generator);
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
    Adj = Eigen::MatrixXd::Identity(LI_state.dimP(),LI_state.dimP());
    Adj.block(0,0,LI_state.dimP()-LI_state.dimTheta(),LI_state.dimP()-LI_state.dimTheta()) = Adjoint_SEK3(LI_state.getX()); 
    cout << "Difference between right invariant covariance (left is mapped using adjoint): \n" << (RI_state.getP() - (Adj * LI_state.getP() * Adj.transpose()).eval()).norm() << endl << endl;
    Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(RI_state.dimP(),RI_state.dimP());
    AdjInv.block(0,0,RI_state.dimP()-RI_state.dimTheta(),RI_state.dimP()-RI_state.dimTheta()) = Adjoint_SEK3(RI_state.Xinv()); 
    cout << "Difference between left invariant covariance (right is mapped using adjoint inverse): \n" << (LI_state.getP() - (AdjInv * RI_state.getP() * AdjInv.transpose()).eval()).norm() << endl << endl;
    cout << "Difference between state estimates: \n" << (LI_state.getX() - RI_state.getX()).norm() << endl << endl;

    // ----- Correct using random data ------
    cout << "\n\n ------ Correct using random data -------\n\n";
    // Set filter's contact state
    vector<pair<int,bool> > contacts;
    contacts.push_back(pair<int,bool> (0, true));
    contacts.push_back(pair<int,bool> (1, true));
    LI_filter.setContacts(contacts);
    RI_filter.setContacts(contacts);

    // Correct state using kinematic measurements
    const int NUM_CORRECT = 10;
    for (int i=0; i<NUM_CORRECT; ++i) {
        vectorKinematics measured_kinematics;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Matrix<double,6,6> covariance = Eigen::Matrix<double,6,6>::Identity();
        p <<  distribution(generator), distribution(generator), distribution(generator);
        pose.block<3,1>(0,3) = p;
        measured_kinematics.push_back(Kinematics(0, pose, covariance));
        p <<  distribution(generator), distribution(generator), distribution(generator);
        pose.block<3,1>(0,3) = p;
        measured_kinematics.push_back(Kinematics(1, pose, covariance));
        LI_filter.CorrectKinematics(measured_kinematics);
        RI_filter.CorrectKinematics(measured_kinematics);
    }

    // Print covariances
    LI_state = LI_filter.getState();
    RI_state = RI_filter.getState();
    cout << "Left Invariant State: \n" << LI_state << endl;
    cout << "Right Invariant State: \n" << RI_state << endl;
    cout << "Left Invariant Covariance: \n" << LI_state.getP() << endl << endl;
    cout << "Right Invariant Covariance: \n" << RI_state.getP() << endl << endl;
    Adj = Eigen::MatrixXd::Identity(LI_state.dimP(),LI_state.dimP());
    Adj.block(0,0,LI_state.dimP()-LI_state.dimTheta(),LI_state.dimP()-LI_state.dimTheta()) = Adjoint_SEK3(LI_state.getX()); 
    cout << "Difference between right invariant covariance (left is mapped using adjoint): \n" << (RI_state.getP() - (Adj * LI_state.getP() * Adj.transpose()).eval()).norm() << endl << endl;
    AdjInv = Eigen::MatrixXd::Identity(RI_state.dimP(),RI_state.dimP());
    AdjInv.block(0,0,RI_state.dimP()-RI_state.dimTheta(),RI_state.dimP()-RI_state.dimTheta()) = Adjoint_SEK3(RI_state.Xinv()); 
    cout << "Difference between left invariant covariance (right is mapped using adjoint inverse): \n" << (LI_state.getP() - (AdjInv * RI_state.getP() * AdjInv.transpose()).eval()).norm() << endl << endl;
    cout << "Difference between state estimates: \n" << (LI_state.getX() - RI_state.getX()).norm() << endl << endl;


    return 0;
}
