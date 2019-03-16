/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.cpp
 *  @author Ross Hartley
 *  @brief  Source file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#include "LieGroup.h"

namespace inekf {

using namespace std;

const double TOLERANCE = 1e-10;

long int factorial(int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    // Convert vector to skew-symmetric matrix
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -v[2], v[1],
         v[2], 0, -v[0], 
        -v[1], v[0], 0;
        return M;
}

Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, int m) {
    // Computes mth integral of the exponential map: \Gamma_m = \sum_{n=0}^{\infty} \dfrac{1}{(n+m)!} (w^\wedge)^n
    assert(m>=0);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta < TOLERANCE) {
        return (1.0/factorial(m))*I; // TODO: There is a better small value approximation for exp() given in Trawny p.19
    } 
    Eigen::Matrix3d A = skew(w);
    double theta2 =  theta*theta;

    // Closed form solution for the first 3 cases
    switch (m) {
        case 0: // Exp map of SO(3)
            return I + (sin(theta)/theta)*A + ((1-cos(theta))/theta2)*A*A;
        
        case 1: // Left Jacobian of SO(3)
            // eye(3) - A*(1/theta^2) * (R - eye(3) - A);
            // eye(3) + (1-cos(theta))/theta^2 * A + (theta-sin(theta))/theta^3 * A^2;
            return I + ((1-cos(theta))/theta2)*A + ((theta-sin(theta))/(theta2*theta))*A*A;

        case 2: 
            // 0.5*eye(3) - (1/theta^2) * (R - eye(3) - A - 0.5*A^2);
            // 0.5*eye(3) + (theta-sin(theta))/theta^3 * A + (2*(cos(theta)-1) + theta^2)/(2*theta^4) * A^2
            return 0.5*I + (theta-sin(theta))/(theta2*theta)*A + (theta2 + 2*cos(theta)-2)/(2*theta2*theta2)*A*A;

        default: // General case 
            Eigen::Matrix3d R = I + (sin(theta)/theta)*A + ((1-cos(theta))/theta2)*A*A;
            Eigen::Matrix3d S = I;
            Eigen::Matrix3d Ak = I;
            long int kfactorial = 1;
            for (int k=1; k<=m; ++k) {
                kfactorial = kfactorial*k;
                Ak = (Ak*A).eval();
                S = (S + (1.0/kfactorial)*Ak).eval();
            }
            if (m==0) { 
                return R;
            } else if (m%2){ // odd 
                return (1.0/kfactorial)*I + (pow(-1,(m+1)/2)/pow(theta,m+1))*A * (R - S);
            } else { // even
                return (1.0/kfactorial)*I + (pow(-1,m/2)/pow(theta,m)) * (R - S);
            }
    }

}

Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w) {
    // Computes the vectorized exponential map for SO(3)
    return Gamma_SO3(w, 0);
}

Eigen::Matrix3d LeftJacobian_SO3(const Eigen::Vector3d& w) {
    // Computes the Left Jacobian of SO(3)
    return Gamma_SO3(w, 1);
}

Eigen::Matrix3d RightJacobian_SO3(const Eigen::Vector3d& w) {
    // Computes the Right Jacobian of SO(3)
    return Gamma_SO3(-w, 1);
}

Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v) {
    // Computes the vectorized exponential map for SE_K(3)
    int K = (v.size()-3)/3;
    Eigen::MatrixXd X = Eigen::MatrixXd::Identity(3+K,3+K);
    Eigen::Matrix3d R;
    Eigen::Matrix3d Jl;
    Eigen::Vector3d w = v.head(3);
    double theta = w.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    if (theta < TOLERANCE) {
        R = I;
        Jl = I;
    } else {
        Eigen::Matrix3d A = skew(w);
        double theta2 = theta*theta;
        double stheta = sin(theta);
        double ctheta = cos(theta);
        double oneMinusCosTheta2 = (1-ctheta)/(theta2);
        Eigen::Matrix3d A2 = A*A;
        R =  I + (stheta/theta)*A + oneMinusCosTheta2*A2;
        Jl = I + oneMinusCosTheta2*A + ((theta-stheta)/(theta2*theta))*A2;
    }
    X.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        X.block<3,1>(0,3+i) = Jl * v.segment<3>(3+3*i);
    }
    return X;
}

Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X) {
    // Compute Adjoint(X) for X in SE_K(3)
    int K = X.cols()-3;
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(3+3*K, 3+3*K);
    Eigen::Matrix3d R = X.block<3,3>(0,0);
    Adj.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        Adj.block<3,3>(3+3*i,3+3*i) = R;
        Adj.block<3,3>(3+3*i,0) = skew(X.block<3,1>(0,3+i))*R;
    }
    return Adj;
}


} // end inekf namespace