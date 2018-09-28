/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   correction_speed.cpp
 *  @author Ross Hartley
 *  @brief  Test to determine average correction speed
 *  @date   September 25, 2018
 **/

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include "InEKF.h"

#define DT_MIN 1e-6
#define DT_MAX 1

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

using namespace std;
using namespace inekf;

typedef vector<pair<double,Eigen::Matrix<double,6,1>>> vectorPairIntVector6d;

int main() {
    // Initialize filter
    InEKF filter;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;

    ifstream infile("../src/data/correction_speed_test_data.txt");
    string line;
    Eigen::Matrix<double,6,1> m, m_last; 
    double t, t_last;
    m_last << 0,0,0,0,0,0;
    t_last = 0;
    vectorPairIntVector6d measurements_vec;
    vectorPairIntVector3d measured_landmarks;

    // Loop through data file and read in measurements line by line
    while (getline(infile, line)){
        vector<string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));
        // Handle measurements
        if (measurement[0].compare("IMU")==0){
            t = stod(measurement[1]); 
            m << stod(measurement[2]), 
                 stod(measurement[3]), 
                 stod(measurement[4]),
                 stod(measurement[5]),
                 stod(measurement[6]),
                 stod(measurement[7]);
            measurements_vec.push_back(pair<double,Eigen::Matrix<double,6,1>> (t, m));

        }
        else if (measurement[0].compare("LANDMARK")==0){
            t = stod(measurement[1]); 
            for (int i=2; i<measurement.size(); i+=4) {
                int landmark_id = stod(measurement[i]);
                Eigen::Vector3d p_bl;
                p_bl << stod(measurement[i+1]), 
                        stod(measurement[i+2]), 
                        stod(measurement[i+3]);
                measured_landmarks.push_back(pair<int,Eigen::Vector3d> (landmark_id, p_bl)); 
            }
        }
    }

    // Propagate all IMU data
    cout << "Propagating " << measurements_vec.size() << " IMU measurements...\n";
    for (auto it=measurements_vec.begin(); it!=measurements_vec.end(); ++it) {
        // Propagate using IMU data
        t = it->first;
        m = it->second;
        double dt = t - t_last;
        if (dt > DT_MIN && dt < DT_MAX) {
            filter.Propagate(m_last, dt);
        }
        // Store previous timestamp
        t_last = t;
        m_last = m;
    }
    cout << filter.getState() << endl;

    // Correct using landmark data
    cout << "Correcting " << measured_landmarks.size() << " landmark measurements...\n";
    int64_t max_duration = 0;
    int64_t sum_duration = 0;
    for (auto it=measured_landmarks.begin(); it!=measured_landmarks.end(); ++it) {
        vectorPairIntVector3d landmarks;
        landmarks.push_back(*it);
        chrono::high_resolution_clock::time_point start_time = chrono::high_resolution_clock::now();
        filter.CorrectLandmarks(landmarks);
        //cout << filter.getState() << endl;
        chrono::high_resolution_clock::time_point end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>( end_time - start_time ).count();
        //cout << "duration: " <<  duration << endl;
        sum_duration += duration;
        if (duration > max_duration)
            max_duration = duration;
    }
    cout << filter.getState() << endl;
    cout << "max duration: " <<  max_duration << endl;
    cout << "average duration: " <<  double(sum_duration)/(measured_landmarks.size()/3) << endl;
    return 0;
}