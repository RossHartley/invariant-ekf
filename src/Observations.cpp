#include "Observations.h"

namespace inekf {

// Default constructor
Observation::Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI) :
    Y(Y), b(b), H(H), N(N), PI(PI) {}

// Check if empty
bool Observation::empty() { return Y.rows() == 0; }

std::ostream& operator<<(std::ostream& os, const Observation& o) {
    os << "---------- Observation ------------" << std::endl;
    os << "Y:\n" << o.Y << std::endl << std::endl;
    os << "b:\n" << o.b << std::endl << std::endl;
    os << "H:\n" << o.H << std::endl << std::endl;
    os << "N:\n" << o.N << std::endl << std::endl;
    os << "PI:\n" << o.PI << std::endl;
    os << "-----------------------------------";
    return os;  
} 

} // end inekf namespace
