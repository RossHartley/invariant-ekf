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
#include <Eigen/StdVector>
#include <iostream>
#include <vector>
#include <map>
#include <tuple>
#if INEKF_USE_MUTEX
#include <mutex>
#endif
#include <algorithm>
#include "RobotState.h"
#include "NoiseParams.h"
#include "LieGroup.h"

namespace inekf {

typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d>>> mapIntVector3d;
typedef std::vector<std::pair<int,Eigen::Vector3d>, Eigen::aligned_allocator<std::pair<int,Eigen::Vector3d>>> vectorPairIntVector3d;
typedef std::vector<std::tuple<int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>>, Eigen::aligned_allocator<std::tuple<int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>>>> vectorTupleIntMatrix4dMatrix6d;
typedef std::vector<std::tuple<int,int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>>, Eigen::aligned_allocator<std::tuple<int,int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>>>> vectorTupleIntIntMatrix4dMatrix6d;

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
        void setContacts(std::vector<std::pair<int,bool>> contacts);

        void Propagate(const Eigen::Matrix<double,6,1>& m, double dt);
        void Correct(const Observation& obs);
        void CorrectLandmarks(const vectorPairIntVector3d& measured_landmarks);
        void CorrectKinematics(const vectorTupleIntMatrix4dMatrix6d& measured_kinematics);
        // TODO: Kinematics between two contact points (useful to prevent double counting if you want multiple contacts per foot)
        // TODO: void CorrectKinematics(const vectorTupleIntIntMatrix4dMatrix6d& measured_kinematics); 

    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_ = (Eigen::VectorXd(3) << 0,0,-9.81).finished(); // Gravity
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
