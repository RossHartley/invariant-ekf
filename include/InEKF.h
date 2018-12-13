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

        /**
         * Resets the filter
         * Initializes state matrix to identity, removes all augmented states, and assigns default noise parameters.
         */
        void clear();

        /**
         * Gets the current state estimate.
         */
        RobotState getState() const;

        /**
         * Gets the current noise parameters.
         */
        NoiseParams getNoiseParams() const;

        /**
         * Gets the filter's current contact states.
         * @return  map of contact ID and bool that indicates if contact is registed
         */
        std::map<int,bool> getContacts() const;

        /**
         * Gets the current estimated contact positions.
         * @return  map of contact ID and associated index in the state matrix X
         */
        std::map<int,int> getEstimatedContactPositions() const;

        /**
         * Gets the filter's prior landmarks.
         * @return  map of prior landmark ID and position (as a Eigen::Vector3d)
         */
        mapIntVector3d getPriorLandmarks() const;

        /**
         * Gets the filter's estimated landmarks.
         * @return  map of landmark ID and associated index in the state matrix X
         */
        std::map<int,int> getEstimatedLandmarks() const;

        /**
         * Gets the filter's set magnetic field.
         * @return  magnetic field in world frame
         */
        Eigen::Vector3d getMagneticField() const;

        /**
         * Sets the current state estimate
         * @param   state estimate
         */
        void setState(RobotState state);
        void setNoiseParams(NoiseParams params);
        void setContacts(std::vector<std::pair<int,bool> > contacts);
        void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
        void setMagneticField(Eigen::Vector3d& true_magnetic_field);

        // Inertial/Contact propagation function
        void Propagate(const Eigen::Matrix<double,6,1>& m, double dt);

        // Right Invariant Measurements
        void CorrectKinematics(const vectorKinematics& measured_kinematics);
        void CorrectLandmarks(const vectorLandmarks& measured_landmarks);
        void CorrectMagnetometer(const Eigen::Vector3d& measured_magnetic_field, const Eigen::Matrix3d& covariance);

        // Left Invariant Measurements
        void CorrectPosition(const Eigen::Vector3d& measured_position, const Eigen::Matrix3d& covariance, const Eigen::Vector3d& indices);
        void CorrectContactPosition(const int id, const Eigen::Vector3d& measured_contact_position, const Eigen::Matrix3d& covariance, const Eigen::Vector3d& indices);

        // Other
        void RemovePriorLandmarks(const int landmark_id);
        void RemovePriorLandmarks(const std::vector<int> landmark_ids);
        void RemoveLandmarks(const int landmark_id);
        void RemoveLandmarks(const std::vector<int> landmark_ids);


    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity
        std::map<int,bool> contacts_;
        std::map<int,int> estimated_contact_positions_;
        mapIntVector3d prior_landmarks_;
        std::map<int,int> estimated_landmarks_;
        Eigen::Vector3d magnetic_field_;

        // Corrects state using invariant observation models
        void CorrectRightInvariant(const Observation& obs);
        void CorrectLeftInvariant(const Observation& obs);
        // void CorrectFullState(const Observation& obs); // TODO
};

} // end inekf namespace
#endif 
