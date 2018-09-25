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
#include <mutex>
#include <algorithm>
#include "RobotState.h"
#include "LieGroup.h"

namespace inekf {

typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d>>> mapIntVector3d;
typedef std::vector<std::pair<int,Eigen::Vector3d>, Eigen::aligned_allocator<std::pair<int,Eigen::Vector3d>>> vectorPairIntVector3d;
typedef std::vector<std::tuple<int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>>, Eigen::aligned_allocator<std::tuple<int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>>>> vectorTupleIntMatrix4dMatrix6d;

class NoiseParams {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NoiseParams();

        void setGyroscopeNoise(double std);
        void setGyroscopeNoise(const Eigen::Vector3d& std);
        void setGyroscopeNoise(const Eigen::Matrix3d& cov);

        void setAccelerometerNoise(double std);
        void setAccelerometerNoise(const Eigen::Vector3d& std);
        void setAccelerometerNoise(const Eigen::Matrix3d& cov);  

        void setGyroscopeBiasNoise(double std);
        void setGyroscopeBiasNoise(const Eigen::Vector3d& std);
        void setGyroscopeBiasNoise(const Eigen::Matrix3d& cov);

        void setAccelerometerBiasNoise(double std);
        void setAccelerometerBiasNoise(const Eigen::Vector3d& std);
        void setAccelerometerBiasNoise(const Eigen::Matrix3d& cov);  

        void setLandmarkNoise(double std);
        void setLandmarkNoise(const Eigen::Vector3d& std);
        void setLandmarkNoise(const Eigen::Matrix3d& cov);

        void setContactNoise(double std);
        void setContactNoise(const Eigen::Vector3d& std);
        void setContactNoise(const Eigen::Matrix3d& cov);

        Eigen::Matrix3d getGyroscopeCov();
        Eigen::Matrix3d getAccelerometerCov();
        Eigen::Matrix3d getGyroscopeBiasCov();
        Eigen::Matrix3d getAccelerometerBiasCov();
        Eigen::Matrix3d getLandmarkCov();
        Eigen::Matrix3d getContactCov();

        friend std::ostream& operator<<(std::ostream& os, const NoiseParams& p);  

    private:
        Eigen::Matrix3d Qg_;
        Eigen::Matrix3d Qa_;
        Eigen::Matrix3d Qbg_;
        Eigen::Matrix3d Qba_;
        Eigen::Matrix3d Ql_;
        Eigen::Matrix3d Qc_;
};




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
        InEKF(RobotState state);
        InEKF(RobotState state, const mapIntVector3d& prior_landmarks);

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

    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_ = (Eigen::VectorXd(3) << 0,0,-9.81).finished(); // Gravity
        mapIntVector3d prior_landmarks_;
        std::map<int,int> estimated_landmarks_;
        std::mutex estimated_landmarks_mutex_;
        std::map<int,bool> contacts_;
        std::map<int,int> estimated_contact_positions_;
        std::mutex estimated_contacts_mutex_;
};

} // end inekf namespace
#endif 
