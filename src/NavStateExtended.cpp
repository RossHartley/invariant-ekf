/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NavStateExtended.cpp
 *  @author Ross Hartley
 *  @brief  Source file for NavStateExtended
 *  @date   May 23, 2019
 **/

#include "NavStateExtended.h"

namespace inekf {

using namespace std;

// Default constructor
NavStateExtended::NavStateExtended() : NavState() {}

// Initialize with provided state
NavStateExtended::NavStateExtended(const Eigen::Matrix3d& R, const Eigen::Vector3d& v, const Eigen::Vector3d& p, 
                   const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) : NavState(R, v, p, bg, ba) {}

// Print
ostream& operator<<(ostream& os, const NavStateExtended& s) {  
    os << "--------- NavState -------------" << endl;
    os << "X:\n" << s.getX() << endl << endl;
    os << "bias: " << s.getBias().transpose() << endl;
    os << "-----------------------------------\n";
    return os;  
} 

} // end inekf namespace


