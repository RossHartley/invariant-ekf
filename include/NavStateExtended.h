/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NavStateExtended.h
 *  @author Ross Hartley
 *  @brief  Header file for NavStateExtended
 *  @date   May 23, 2019
 **/

#ifndef NAVSTATEEXTENDED_H
#define NAVSTATEEXTENDED_H 
#include "NavState.h"

namespace inekf {

class NavStateExtended : public NavState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NavStateExtended();
        NavStateExtended(const Eigen::Matrix3d& R, const Eigen::Vector3d& v, const Eigen::Vector3d& p, 
                 const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);

        friend std::ostream& operator<<(std::ostream& os, const NavStateExtended& s);   

    private:

};

} // end inekf namespace
#endif // NAVSTATEEXTENDED_H

