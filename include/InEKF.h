/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#ifndef INEKF_H
#define INEKF_H 
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#if INEKF_USE_MUTEX
#include <mutex>
#endif
#include <algorithm>
#include "RobotState.h"
#include "NoiseParams.h"
#include "LieGroup.h"

namespace inekf {

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
        Landmark(int id_in, Eigen::Vector3d position_in) : id(id_in), position(position_in) { }

        int id;
        Eigen::Vector3d position;
};

typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > > mapIntVector3d;
typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > >::iterator mapIntVector3dIterator;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > vectorLandmarks;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> >::const_iterator vectorLandmarksIterator;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> > vectorKinematics;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> >::const_iterator vectorKinematicsIterator;

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


class InEKF {
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        InEKF();
        InEKF(NoiseParams params);
        InEKF(RobotState state);
        InEKF(RobotState state, NoiseParams params);

        RobotState getState();
        NoiseParams getNoiseParams();
        mapIntVector3d getPriorLandmarks();
        std::map<int,int> getEstimatedLandmarks();
        std::map<int,bool> getContacts();
        std::map<int,int> getEstimatedContactPositions();
        void setState(RobotState state);
        void setNoiseParams(NoiseParams params);
        void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
        void setContacts(std::vector<std::pair<int,bool> > contacts);

        void Propagate(const Eigen::Matrix<double,6,1>& m, double dt);
        void Correct(const Observation& obs);
        void CorrectLandmarks(const vectorLandmarks& measured_landmarks);
        void CorrectKinematics(const vectorKinematics& measured_kinematics);

    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity
        mapIntVector3d prior_landmarks_;
        std::map<int,int> estimated_landmarks_;
        std::map<int,bool> contacts_;
        std::map<int,int> estimated_contact_positions_;
#if INEKF_USE_MUTEX
        std::mutex estimated_contacts_mutex_;
        std::mutex estimated_landmarks_mutex_;
#endif
};

} // end inekf namespace
#endif 
