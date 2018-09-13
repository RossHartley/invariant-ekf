#ifndef LIEGROUP_H
#define LIEGROUP_H 
#include <Eigen/Dense>
#include <iostream>

namespace inekf {

#define TOLERANCE 1e-10 

Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);
Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v);
Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);

} // end inekf namespace
#endif 
