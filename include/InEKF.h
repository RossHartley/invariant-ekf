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
#include <algorithm>
#include "RobotState.h"
#include "NoiseParams.h"
#include "LieGroup.h"
#include "Observations.h"

namespace inekf {

class InEKF {
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Constructors
        InEKF();
        InEKF(NoiseParams params);
        InEKF(RobotState state);
        InEKF(RobotState state, NoiseParams params);

        // Reset filter
        void clear();

        // Getters
        RobotState getState() const;
        NoiseParams getNoiseParams() const;
        mapIntVector3d getPriorLandmarks() const;
        std::map<int,int> getEstimatedLandmarks() const;
        std::map<int,bool> getContacts() const;
        std::map<int,int> getEstimatedContactPositions() const;

        // Setters
        void setState(RobotState state);
        void setNoiseParams(NoiseParams params);
        void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
        void setContacts(std::vector<std::pair<int,bool> > contacts);

        // Inertial/Contact propagation function
        void Propagate(const Eigen::Matrix<double,6,1>& m, double dt);

        // Right Invariant Measurements
        void CorrectKinematics(const vectorKinematics& measured_kinematics);
        void CorrectLandmarks(const vectorLandmarks& measured_landmarks);
        void CorrectMagnetometer(const Eigen::Vector3d& measured_magnetic_field, const Eigen::Vector3d& true_magnetic_field);

        // Left Invariant Measurements
        void CorrectPosition(const Eigen::Vector3d& measured_position, const Eigen::Vector3d& indices);
        void CorrectContactPositions(const mapIntVector3d& measured_contact_positions, const Eigen::Vector3d& indices);

    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity
        std::map<int,bool> contacts_;
        std::map<int,int> estimated_contact_positions_;
        mapIntVector3d prior_landmarks_;
        std::map<int,int> estimated_landmarks_;

        // Corrects state using invariant observation models
        void CorrectRightInvariant(const Observation& obs);
        void CorrectLeftInvariant(const Observation& obs);
        // void CorrectFullState(const Observation& obs); // TODO
};

} // end inekf namespace
#endif 
