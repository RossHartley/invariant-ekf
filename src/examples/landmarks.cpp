#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include "InEKF.h"

#define DT_MIN 1e-6
#define DT_MAX 1

using namespace std;
using namespace inekf;

int main() 
{
    // Initialize robot's state
    RobotState initial_state; 

    // Initialize prior landmarks
    mapIntVector3d prior_landmarks;
    Eigen::Vector3d p_wl;
    int id;

    id = 0;
    p_wl << 1,2,3;
    prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    id = 1;
    p_wl << 4,5,6;
    prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    id = 2;
    p_wl << 7,8,9;
    prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 
    
    // Initialize filter
    InEKF filter(initial_state, prior_landmarks);
    cout << filter.getNoiseParams() << endl;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;

    ifstream infile("../src/data/landmark_measurements.txt");
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
            cout << "m:\n" << m << endl;
            double dt = t - t_last;
            cout << "dt: " << dt << endl;
            if (dt > DT_MIN && dt < DT_MAX) {
                filter.Propagate(m_last, dt);
                cout << filter.getState() << endl;
            }

        }
        else if (measurement[0].compare("LANDMARK")==0){
            cout << "Received LANDMARK observation, correcting state\n";
            t = stod(measurement[1]); 
            vectorPairIntVector3d measured_landmarks;
            for (int i=2; i<measurement.size(); i+=4) {
                int landmark_id = stod(measurement[i]);
                Eigen::Vector3d p_bl;
                p_bl << stod(measurement[i+1]), 
                        stod(measurement[i+2]), 
                        stod(measurement[i+3]);
                measured_landmarks.push_back(pair<int,Eigen::Vector3d> (landmark_id, p_bl)); 
            }

            // Correct state using landmark measurements
            filter.CorrectLandmarks(measured_landmarks);
            cout << filter.getState() << endl;

        }

        // Store previous timestamp
        t_last = t;
        m_last = m;
    }

    return 0;
}