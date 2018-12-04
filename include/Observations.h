/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Observations.h
 *  @author Ross Hartley
 *  @brief  Header file for Observations
 *  @date   December 03, 2018
 **/

#ifndef OBSERVATIONS_H
#define OBSERVATIONS_H 
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>

namespace inekf {

// Simple class to hold general observations 
class Observation {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI);
        bool empty();

        Eigen::VectorXd Y;
        Eigen::VectorXd b;
        Eigen::MatrixXd H;
        Eigen::MatrixXd N;
        Eigen::MatrixXd PI;

        friend std::ostream& operator<<(std::ostream& os, const Observation& o);  
};


class Kinematics {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Kinematics(int id_in, Eigen::Matrix4d pose_in, Eigen::Matrix<double,6,6> covariance_in) : id(id_in), pose(pose_in), covariance(covariance_in) { }

        int id;
        Eigen::Matrix4d pose;
        Eigen::Matrix<double,6,6> covariance;
};


class Landmark {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Landmark(int id_in, Eigen::Vector3d position_in, Eigen::Matrix3d covariance_in) : id(id_in), position(position_in), covariance(covariance_in) { }

        int id;
        Eigen::Vector3d position;
        Eigen::Matrix3d covariance;
};




// Useful types
typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > > mapIntVector3d;
typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > >::const_iterator mapIntVector3dIterator;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> > vectorKinematics;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> >::const_iterator vectorKinematicsIterator;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > vectorLandmarks;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> >::const_iterator vectorLandmarksIterator;


} // end inekf namespace
#endif // end OBSERVATIONS_H
