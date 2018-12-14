/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   large_state_speed.cpp
 *  @author Ross Hartley
 *  @brief  Tests the speed of the InEKF propagation and correction algorithms when the state is large
 *  @date   September 25, 2018
 **/

#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include "InEKF.h"

#define DT_MIN 1e-6
#define DT_MAX 1

using namespace std;
using namespace inekf;

int main() {
    // Initialize filter
    InEKF filter;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;

    const int NUM_FRAMES = 100;
     chrono::high_resolution_clock::time_point start_time = chrono::high_resolution_clock::now();
    for (int i=0; i<NUM_FRAMES; ++i) {
        cout << "Frame: " << i << endl;

        // Add lots of landmarks
        const int NUM_LANDMARKS = 30;
        vectorLandmarks measured_landmarks;
        Eigen::Matrix3d cov = 0.01*Eigen::Matrix3d::Identity();
        for (int i=0; i<NUM_LANDMARKS; ++i) {
            int id = i;
            Eigen::Vector3d p;
            p << i,i,i;
            Landmark landmark(id,p,cov);
            measured_landmarks.push_back(landmark);
        } 
        filter.CorrectLandmarks(measured_landmarks);
        // cout << filter.getState() << endl;

        // Integrate IMU data
        const int NUM_IMU = 30;
        double dt = 0.01;
        Eigen::Matrix<double,6,1> imu;
        for (int i=0; i<NUM_IMU; ++i) {
            imu << 1,2,3,4,5,6;
            filter.Propagate(imu, dt);
        }
        // cout << filter.getState() << endl;
    }
    chrono::high_resolution_clock::time_point end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::seconds>( end_time - start_time ).count();
    cout << "average frequency: " << 1.0/(double(duration)/double(NUM_FRAMES)) << " Hz" << endl;

    return 0;
}
