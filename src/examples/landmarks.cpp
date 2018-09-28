/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   landmarks.cpp
 *  @author Ross Hartley
 *  @brief  Example of invariant filtering for landmark-aided inertial navigation
 *  @date   September 25, 2018
 **/

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include "InEKF.h"

#define DT_MIN 1e-6
#define DT_MAX 1

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
          0, 0, -1;
    v0 << 0,0,0; // initial velocity
    p0 << 0,0,0; // initial position
    bg0 << 0,0,0; // initial gyroscope bias
    ba0 << 0,0,0; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.01);
    noise_params.setAccelerometerNoise(0.1);
    noise_params.setGyroscopeBiasNoise(0.00001);
    noise_params.setAccelerometerBiasNoise(0.0001);
    noise_params.setLandmarkNoise(0.1);

    // Initialize filter
    InEKF filter(initial_state, noise_params);
    cout << "Noise parameters are initialized to: \n";
    cout << filter.getNoiseParams() << endl;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;

    // --- Optionally initialize prior landmarks --- //
    mapIntVector3d prior_landmarks;
    Eigen::Vector3d p_wl;
    int id;

    // Landmark 1
    id = 1;
    p_wl << 0,-1,0;
    prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // // Landmark 2
    // id = 2;
    // p_wl << 1,1,-0.5;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // Landmark 3
    id = 3;
    p_wl << 2,-1,0.5;
    prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // Store landmarks for localization
    filter.setPriorLandmarks(prior_landmarks); 


    // Open data file
    ifstream infile("../src/data/imu_landmark_measurements.txt");
    string line;
    Eigen::Matrix<double,6,1> imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();
    double t = 0;
    double t_prev = 0;

    // Loop through data file and read in measurements line by line
    while (getline(infile, line)){
        vector<string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));
        // Handle measurements
        if (measurement[0].compare("IMU")==0){
            cout << "Received IMU Data, propagating state\n";
            assert((measurement.size()-2) == 6);
            t = stod(measurement[1]); 
            imu_measurement << stod(measurement[2]), 
                               stod(measurement[3]), 
                               stod(measurement[4]),
                               stod(measurement[5]),
                               stod(measurement[6]),
                               stod(measurement[7]);

            // Propagate using IMU data
            double dt = t - t_prev;
            if (dt > DT_MIN && dt < DT_MAX) {
                filter.Propagate(imu_measurement_prev, dt);
            }
        }
        else if (measurement[0].compare("LANDMARK")==0){
            cout << "Received LANDMARK observation, correcting state\n";
            assert((measurement.size()-2)%4 == 0);
            t = stod(measurement[1]); 
            vectorPairIntVector3d measured_landmarks;
            for (int i=2; i<measurement.size(); i+=4) {
                int landmark_id = stod(measurement[i]);
                Eigen::Vector3d p_bl;
                p_bl << stod(measurement[i+1]), 
                        stod(measurement[i+2]), 
                        stod(measurement[i+3]);
                measured_landmarks.push_back(pair<int,Eigen::Vector3d> (landmark_id, p_bl)); 
            }

            // Correct state using landmark measurements
            filter.CorrectLandmarks(measured_landmarks);
        }

        // Store previous timestamp
        t_prev = t;
        imu_measurement_prev = imu_measurement;
    }

    // Print final state
    cout << filter.getState() << endl;
    return 0;
}