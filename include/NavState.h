/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NavState.h
 *  @author Ross Hartley
 *  @brief  Header file for NavState
 *  @date   May 23, 2019
 **/

#ifndef NAVSTATE_H
#define NAVSTATE_H 
#include <Eigen/Dense>
#include <iostream>

namespace inekf {

class NavState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NavState();
        NavState(const Eigen::Matrix3d& R, const Eigen::Vector3d& v, const Eigen::Vector3d& p, 
                 const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);

        const Eigen::Matrix3d getRotation() const;
        const Eigen::Vector3d getVelocity() const;
        const Eigen::Vector3d getPosition() const;
        const Eigen::Vector3d getGyroscopeBias() const;
        const Eigen::Vector3d getAccelerometerBias() const;

        const Eigen::Matrix<double,5,5> getX() const;
        const Eigen::Matrix<double,6,1> getBias() const;

        void setRotation(const Eigen::Matrix3d& R);
        void setVelocity(const Eigen::Vector3d& v);
        void setPosition(const Eigen::Vector3d& p);
        void setGyroscopeBias(const Eigen::Vector3d& bg);
        void setAccelerometerBias(const Eigen::Vector3d& ba);

        const Eigen::Matrix<double,5,5> setX(const Eigen::Matrix<double,5,5>& X);
        const Eigen::Matrix<double,6,1> setBias(const Eigen::Matrix<double,6,1>& bias);

        const int dimX() const;
        const int dimBias() const;

        NavState inverse() const;

        friend std::ostream& operator<<(std::ostream& os, const NavState& s);  

    private:
        Eigen::Matrix3d R_;
        Eigen::Vector3d v_;
        Eigen::Vector3d p_;
        Eigen::Vector3d bg_;
        Eigen::Vector3d ba_;

        const double dimX_ = 9;
        const double dimBias_ = 6;
};

} // end inekf namespace
#endif // NAVSTATE_H

