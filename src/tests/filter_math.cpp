/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   filter_math.cpp
 *  @author Ross Hartley
 *  @brief  Test to compare filtering results with original MATLAB implementation
 *  @date   September 25, 2018
 **/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include<Eigen/StdVector>
#include <boost/algorithm/string.hpp>
#include "InEKF.h"

#define DT_MIN 1e-6
#define DT_MAX 1

using namespace std;
using namespace inekf;

typedef vector<pair<double,Eigen::Matrix<double,6,1>>> vectorPairIntVector6d;

int main() {
    // Initialize filter
    InEKF filter;
    cout << filter.getState() << endl;

    // // Add prior landmarks
    // mapIntVector3d prior_landmarks;
    int id;
    // Eigen::Vector3d p_wl;
    // id = 0;
    // p_wl << 1,2,3;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 
    // id = 1;
    // p_wl << 4,5,6;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 
    // id = 2;
    // p_wl << 7,8,9;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 
    // filter.setPriorLandmarks(prior_landmarks);

    // Check Landmark correction
    vectorPairIntVector3d measured_landmarks;
    Eigen::Vector3d p_bl;
    id = 0;
    p_bl << 1,2,3;
    measured_landmarks.push_back(pair<int,Eigen::Vector3d> (id, p_bl)); 
    id = 1;
    p_bl << 4,5,6;
    measured_landmarks.push_back(pair<int,Eigen::Vector3d> (id, p_bl)); 
    id = 2;
    p_bl << 7,8,9;
    measured_landmarks.push_back(pair<int,Eigen::Vector3d> (id, p_bl));   
    filter.CorrectLandmarks(measured_landmarks);
    cout << filter.getState() << endl;

    // Propagate and check results
    Eigen::Matrix<double,6,1> m;
    double dt;
    m << 1,2,3,4,5,6;
    dt = 0.1;
    filter.Propagate(m, dt);
    filter.Propagate(m, dt);
    cout << filter.getState() << endl;

    // Check Landmark correction
    filter.CorrectLandmarks(measured_landmarks);
    cout << filter.getState() << endl;

    return 0;
}

// getP1() {
//     Eigen::Matrix<double,15,15> P;
//     P << 

// }