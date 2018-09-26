/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics.cpp
 *  @author Ross Hartley
 *  @brief  Example of invariant filtering for contact-aided inertial navigation
 *  @date   September 25, 2018
 **/

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <tuple>
#include "InEKF.h"

#define DT_MIN 1e-6
#define DT_MAX 1

using namespace std;
using namespace inekf;

int main() 
{
    // Initialize robot's state
    RobotState initial_state; 
    Eigen::Matrix3d R0;
    // IMU frame is rotated 90deg about the x-axis
    R0 << 1, 0, 0,
          0, -1, 0,
          0, 0, -1;
    initial_state.setRotation(R0);

    // Initialize filter
    InEKF filter(initial_state);
    cout << "Noise parameters are initialized to: \n";
    cout << filter.getNoiseParams() << endl;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;

    ifstream infile("../src/data/kinematic_measurements.txt");
    string line;
    Eigen::Matrix<double,6,1> m, m_last; 
    double t, t_last;
    m_last << 0,0,0,0,0,0;
    t_last = 0;

    // Loop through data file and read in measurements line by line
    while (getline(infile, line)){
        vector<string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));
        // // Handle measurements
        if (measurement[0].compare("IMU")==0){
            cout << "Received IMU Data, propagating state\n";
            t = stod(measurement[1]); 
            m << stod(measurement[2]), 
                 stod(measurement[3]), 
                 stod(measurement[4]),
                 stod(measurement[5]),
                 stod(measurement[6]),
                 stod(measurement[7]);

            // Propagate using IMU data
            double dt = t - t_last;
            if (dt > DT_MIN && dt < DT_MAX) {
                filter.Propagate(m_last, dt);
            }

        }
        else if (measurement[0].compare("CONTACT")==0){
            cout << "Received CONTACT Data, setting filter's contact state\n";
            vector<pair<int,bool>> contacts;
            int id;
            bool indicator;
            t = stod(measurement[1]); 
            // Left foot contact
            id = stod(measurement[2]);
            if (stod(measurement[3]) == 1) {
                indicator = true;
            } else {
                indicator = false;
            }
            contacts.push_back(pair<int,bool> (id, indicator));
            // Right foot contact
            id = stod(measurement[4]);
            if (stod(measurement[5]) == 1) {
                indicator = true;
            } else {
                indicator = false;
            }
            contacts.push_back(pair<int,bool> (id, indicator));
            // Set filter's contact state
            filter.setContacts(contacts);
        }
        else if (measurement[0].compare("KINEMATIC")==0){
            cout << "Received KINEMATIC observation, correcting state\n";  
            int id;
            Eigen::Matrix4d H;
            Eigen::Matrix<double,6,6> Cov;
            vectorTupleIntMatrix4dMatrix6d measured_kinematics;
            t = stod(measurement[1]); 
            // Add left foot kinematics
            id = stod(measurement[2]); 
            H << stod(measurement[3]),stod(measurement[4]),stod(measurement[5]),stod(measurement[6]),
                 stod(measurement[7]),stod(measurement[8]),stod(measurement[9]),stod(measurement[10]),
                 stod(measurement[11]),stod(measurement[12]),stod(measurement[13]),stod(measurement[14]),
                 stod(measurement[15]),stod(measurement[16]),stod(measurement[17]),stod(measurement[18]);
            Cov << stod(measurement[19]),stod(measurement[20]),stod(measurement[21]),stod(measurement[22]),stod(measurement[23]),stod(measurement[24]),
                   stod(measurement[25]),stod(measurement[26]),stod(measurement[27]),stod(measurement[28]),stod(measurement[29]),stod(measurement[30]),
                   stod(measurement[31]),stod(measurement[32]),stod(measurement[33]),stod(measurement[34]),stod(measurement[35]),stod(measurement[36]),
                   stod(measurement[37]),stod(measurement[38]),stod(measurement[39]),stod(measurement[40]),stod(measurement[41]),stod(measurement[42]),
                   stod(measurement[43]),stod(measurement[44]),stod(measurement[45]),stod(measurement[46]),stod(measurement[47]),stod(measurement[48]),
                   stod(measurement[49]),stod(measurement[50]),stod(measurement[51]),stod(measurement[52]),stod(measurement[53]),stod(measurement[54]);
            measured_kinematics.push_back(tuple<int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>> (id, H, Cov));
            // Add right foot kinematics
            id = stod(measurement[55]); 
            H << stod(measurement[56]),stod(measurement[57]),stod(measurement[58]),stod(measurement[59]),
                 stod(measurement[60]),stod(measurement[61]),stod(measurement[62]),stod(measurement[63]),
                 stod(measurement[64]),stod(measurement[65]),stod(measurement[66]),stod(measurement[67]),
                 stod(measurement[68]),stod(measurement[69]),stod(measurement[70]),stod(measurement[71]);
            Cov << stod(measurement[72]),stod(measurement[73]),stod(measurement[74]),stod(measurement[75]),stod(measurement[76]),stod(measurement[77]),
                   stod(measurement[78]),stod(measurement[79]),stod(measurement[80]),stod(measurement[81]),stod(measurement[82]),stod(measurement[83]),
                   stod(measurement[84]),stod(measurement[85]),stod(measurement[86]),stod(measurement[87]),stod(measurement[88]),stod(measurement[89]),
                   stod(measurement[90]),stod(measurement[91]),stod(measurement[92]),stod(measurement[93]),stod(measurement[94]),stod(measurement[95]),
                   stod(measurement[96]),stod(measurement[97]),stod(measurement[98]),stod(measurement[99]),stod(measurement[100]),stod(measurement[101]),
                   stod(measurement[102]),stod(measurement[103]),stod(measurement[104]),stod(measurement[105]),stod(measurement[106]),stod(measurement[107]);
            measured_kinematics.push_back(tuple<int,Eigen::Matrix4d,Eigen::Matrix<double,6,6>> (id, H, Cov));
            // Correct state using kinematic measurements
            filter.CorrectKinematics(measured_kinematics);
        }

        // Store previous timestamp
        t_last = t;
        m_last = m;
    }

    // Print final state
    cout << filter.getState() << endl;
    return 0;
}