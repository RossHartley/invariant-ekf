/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.h
 *  @author Ross Hartley
 *  @brief  Header file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#ifndef LIEGROUP_H
#define LIEGROUP_H 
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

namespace inekf {

extern const double TOLERANCE;

long int factorial(int n);
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, int n);
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);
Eigen::Matrix3d LeftJacobian_SO3(const Eigen::Vector3d& w);
Eigen::Matrix3d RightJacobian_SO3(const Eigen::Vector3d& w);
Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v);
Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);

} // end inekf namespace
#endif 
