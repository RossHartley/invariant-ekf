#include "RobotState.h"
#include "LieGroup.h"
using namespace std;

// Default constructor
RobotState::RobotState() : 
    X_(Eigen::MatrixXd::Identity(5,5)), Theta_(Eigen::MatrixXd::Zero(6,1)), P_(Eigen::MatrixXd::Identity(15,15)) {}
// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd& X) : 
    X_(X), Theta_(Eigen::MatrixXd::Zero(6,1)) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta) : 
    X_(X), Theta_(Theta) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P) : 
    X_(X), Theta_(Theta), P_(P) {}
// TODO: error checking to make sure dimensions are correct and supported


const Eigen::MatrixXd RobotState::getX() { return X_; }
const Eigen::VectorXd RobotState::getTheta() { return Theta_; }
const Eigen::MatrixXd RobotState::getP() { return P_; }
const Eigen::Matrix3d RobotState::getRotation() { return X_.block<3,3>(0,0); }
const Eigen::Vector3d RobotState::getVelocity() { return X_.block<3,1>(0,3); }
const Eigen::Vector3d RobotState::getPosition() { return X_.block<3,1>(0,4); }
const Eigen::Vector3d RobotState::getAngularVelocityBias() { return Theta_.head(3); }
const Eigen::Vector3d RobotState::getLinearAccelerationBias() { return Theta_.tail(3); }
const int RobotState::dimX() { return X_.cols(); }
const int RobotState::dimTheta() { return Theta_.rows(); }
const int RobotState::dimP() { return P_.cols(); }

void RobotState::setX(Eigen::MatrixXd& X) { X_ = X; }
void RobotState::setTheta(Eigen::VectorXd& Theta) { Theta_ = Theta; }
void RobotState::setP(Eigen::MatrixXd& P) { P_ = P; }
void RobotState::setRotation(Eigen::Matrix3d& R) { X_.block<3,3>(0,0) = R; }
void RobotState::setVelocity(Eigen::Vector3d& v) { X_.block<3,1>(0,3) = v; }
void RobotState::setPosition(Eigen::Vector3d& p) { X_.block<3,1>(0,4) = p; }
void RobotState::setAngularVelocityBias(Eigen::Vector3d& bg) { Theta_.head(3) = bg; }
void RobotState::setLinearAccelerationBias(Eigen::Vector3d& ba) { Theta_.tail(3) = ba; }


void RobotState::copyDiagX(int n, Eigen::MatrixXd& BigX) {
    int dimX = this->dimX();
    for(int i=0; i<n; ++i) {
        int startIndex = BigX.rows();
        BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigX.block(startIndex,0,dimX,startIndex) = Eigen::MatrixXd::Zero(dimX,startIndex);
        BigX.block(0,startIndex,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
        BigX.block(startIndex,startIndex,dimX,dimX) = X_;
    }
    return;
}

ostream& operator<<(ostream& os, const RobotState& s) {  
    os << "--------- Robot State -------------" << endl;
    os << "X:\n" << s.X_ << endl << endl;
    os << "Theta:\n" << s.Theta_ << endl << endl;
    os << "P:\n" << s.P_ << endl;
    os << "-----------------------------------";
    return os;  
} 