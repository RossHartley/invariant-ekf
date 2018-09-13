#include "InEKF.h"

namespace inekf {

using namespace std;

// ------------ NoiseParams -------------
// Default Constructor
NoiseParams::NoiseParams() {
    setGyroscopeNoise(0.01);
    setAccelerometerNoise(0.1);
    setGyroscopeBiasNoise(0.00001);
    setAccelerometerBiasNoise(0.0001);
    setLandmarkNoise(0.1);
}

void NoiseParams::setGyroscopeNoise(double std) { Qg_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setGyroscopeNoise(const Eigen::Vector3d& std) { Qg_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setGyroscopeNoise(const Eigen::Matrix3d& cov) { Qg_ = cov; }

void NoiseParams::setAccelerometerNoise(double std) { Qa_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setAccelerometerNoise(const Eigen::Vector3d& std) { Qa_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setAccelerometerNoise(const Eigen::Matrix3d& cov) { Qa_ = cov; } 

void NoiseParams::setGyroscopeBiasNoise(double std) { Qbg_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setGyroscopeBiasNoise(const Eigen::Vector3d& std) { Qbg_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setGyroscopeBiasNoise(const Eigen::Matrix3d& cov) { Qbg_ = cov; }

void NoiseParams::setAccelerometerBiasNoise(double std) { Qbg_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setAccelerometerBiasNoise(const Eigen::Vector3d& std) { Qba_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setAccelerometerBiasNoise(const Eigen::Matrix3d& cov) { Qba_ = cov; }

void NoiseParams::setLandmarkNoise(double std) { Ql_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setLandmarkNoise(const Eigen::Vector3d& std) { Ql_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setLandmarkNoise(const Eigen::Matrix3d& cov) { Ql_ = cov; }

Eigen::Matrix3d NoiseParams::getGyroscopeCov() { return Qg_; }
Eigen::Matrix3d NoiseParams::getAccelerometerCov() { return Qa_; }
Eigen::Matrix3d NoiseParams::getGyroscopeBiasCov() { return Qbg_; }
Eigen::Matrix3d NoiseParams::getAccelerometerBiasCov() { return Qba_; }
Eigen::Matrix3d NoiseParams::getLandmarkCov() { return Ql_; }



// ------------ Observation -------------
// Default constructor
Observation::Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI) :
    Y(Y), b(b), H(H), N(N), PI(PI) {}

// Check if empty
bool Observation::empty() { return Y.rows() == 0; }

ostream& operator<<(ostream& os, const Observation& o) {
    os << "---------- Observation ------------" << endl;
    os << "Y:\n" << o.Y << endl << endl;
    os << "b:\n" << o.b << endl << endl;
    os << "H:\n" << o.H << endl << endl;
    os << "N:\n" << o.N << endl << endl;
    os << "PI:\n" << o.PI << endl;
    os << "-----------------------------------";
    return os;  
} 



// ------------ InEKF -------------
// Default constructor
InEKF::InEKF() {}

// Constructor with initial state
InEKF::InEKF(RobotState state) : state_(state) {}

// Constructor wtih initial state and prior landmarks
InEKF::InEKF(RobotState state, const mapIntVector3d& prior_landmarks) : state_(state), prior_landmarks_(prior_landmarks) {}

// Return robot's current state
RobotState InEKF::getState() { return state_; }

// InEKF Propagation - Inertial Data
void InEKF::Propagate(const Eigen::Matrix<double,6,1>& m, double dt) {
    Eigen::Vector3d w = m.head(3) - state_.getAngularVelocityBias();    // Angular Velocity
    Eigen::Vector3d a = m.tail(3) - state_.getLinearAccelerationBias(); // Linear Acceleration
    
    Eigen::MatrixXd X = state_.getX();
    Eigen::MatrixXd P = state_.getP();

    // Extract State
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d v = state_.getVelocity();
    Eigen::Vector3d p = state_.getPosition();

    // Strapdown IMU motion model
    Eigen::Vector3d phi = w*dt; 
    Eigen::Matrix3d R_pred = R * Exp_SO3(phi);
    Eigen::Vector3d v_pred = v + (R*a + g_)*dt;
    Eigen::Vector3d p_pred = p + v*dt + 0.5*(R*a + g_)*dt*dt;

    // Set new state (bias has constant dynamics)
    state_.setRotation(R_pred);
    state_.setVelocity(v_pred);
    state_.setPosition(p_pred);

    // ---- Linearized invariant error dynamics -----
    int dimX = state_.dimX();
    int dimP = state_.dimP();
    int dimTheta = state_.dimTheta();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimP,dimP);
    // Inertial terms
    A.block<3,3>(3,0) = skew(g_); // TODO: Efficiency could be improved by not computing the constant terms every time
    A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
    // Bias terms
    A.block<3,3>(0,dimP-dimTheta) = -R;
    A.block<3,3>(3,dimP-dimTheta+3) = -R;
    for (int i=3; i<dimX; ++i) {
        A.block<3,3>(3*i-6,dimP-dimTheta) = -skew(X.block<3,1>(0,i))*R;
    } 

    // Noise terms
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(dimP,dimP);
    Qk.block<3,3>(0,0) = noise_params_.getGyroscopeCov(); 
    Qk.block<3,3>(3,3) = noise_params_.getAccelerometerCov();
    Qk.block<3,3>(dimP-dimTheta,dimP-dimTheta) = noise_params_.getGyroscopeBiasCov();
    Qk.block<3,3>(dimP-dimTheta+3,dimP-dimTheta+3) = noise_params_.getAccelerometerBiasCov();

    // Discretization
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dimP,dimP);
    Eigen::MatrixXd Phi = I + A*dt; // Fast approximation of exp(A*dt). TODO: explore using the full exp() instead
    Eigen::MatrixXd Adj = I;
    Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X); // Approx 200 microseconds
    Eigen::MatrixXd PhiAdj = Phi * Adj;
    Eigen::MatrixXd Qk_hat = PhiAdj * Qk * PhiAdj.transpose() * dt; // Approximated discretized noise matrix (faster by 400 microseconds)
    //Eigen::MatrixXd Qk_hat = Phi * Adj * Qk * Adj.transpose() * Phi.transpose() * dt; // Approximated discretized noise matrix 

    // Propagate Covariance
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qk_hat;
    //Eigen::MatrixXd P_pred = Phi * (P + Adj*Qk*Adj.transpose()*dt) * Phi.transpose(); // Faster?

    //cout << "diff: \n" << (P_pred-P_pred2).norm() << endl;
    // Set new covariance
    state_.setP(P_pred);

    return;
}

// Correct State: Right-Invariant Observation
void InEKF::Correct(const Observation& obs) {
    // Compute Kalman Gain
    Eigen::MatrixXd P = state_.getP();
    //cout << "P: \n" << P << endl;
    //cout << "H^T: \n" << obs.H.transpose() << endl;
    Eigen::MatrixXd PHT = P * obs.H.transpose();
    //cout << "PHT: \n" << PHT << endl;
    Eigen::MatrixXd S = obs.H * PHT;
    //cout << "S: \n" << S << endl;
    Eigen::MatrixXd K = PHT * S.inverse();
    //cout << "K: \n" << K << endl;

    // Copy X along the diagonals if more than one measurement
    Eigen::MatrixXd BigX;
    state_.copyDiagX(obs.Y.rows()/state_.dimX(), BigX);
   
    // Compute correction terms
    Eigen::MatrixXd Z = BigX*obs.Y - obs.b;
    Eigen::VectorXd delta = K*obs.PI*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-state_.dimTheta()));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-state_.dimTheta(), state_.dimTheta());
    //cout << "Z: \n" << Z << endl;

    // Update state
    Eigen::MatrixXd X_new = dX*state_.getX(); // Right-Invariant Update
    Eigen::VectorXd Theta_new = state_.getTheta() + dTheta;
    state_.setX(X_new); 
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(state_.dimP(),state_.dimP()) - K*obs.H;
    //cout << "IKH: \n" << IKH << endl;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*obs.N*K.transpose(); // Joseph update form
    //cout << "P_new: \n" << P_new << endl;
    
    state_.setP(P_new); 
}   

// Create Observation from vector of landmark measurements
void InEKF::CorrectLandmarks(const vectorPairIntVector3d& measured_landmarks) {
    Eigen::VectorXd Y;
    Eigen::VectorXd b;
    Eigen::MatrixXd H;
    Eigen::MatrixXd N;
    Eigen::MatrixXd PI;

    Eigen::Matrix3d R = state_.getRotation();
    vectorPairIntVector3d new_landmarks;
    vector<int> used_landmark_ids;
    
    for (auto it=measured_landmarks.begin(); it!=measured_landmarks.end(); ++it) {
        // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
        if (find(used_landmark_ids.begin(), used_landmark_ids.end(), it->first) != used_landmark_ids.end()) { 
            cout << "Duplicate landmark ID detected! Skipping measurement.\n";
            continue; 
        } else { used_landmark_ids.push_back(it->first); }

        // See if we can find id in prior_landmarks or estimated_landmarks
        auto it_prior = prior_landmarks_.find(it->first);
        auto it_estimated = estimated_landmarks_.find(it->first);
        if (it_prior!=prior_landmarks_.end()) {
            // Found in prior landmark set
            int dimX = state_.dimX();
            int dimP = state_.dimP();
            int startIndex;

            // Fill out Y
            startIndex = Y.rows();
            Y.conservativeResize(startIndex+dimX, Eigen::NoChange);
            Y.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
            Y.segment(startIndex,3) = it->second; // p_bl
            Y(startIndex+4) = 1; 

            // Fill out b
            startIndex = b.rows();
            b.conservativeResize(startIndex+dimX, Eigen::NoChange);
            b.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
            b.segment(startIndex,3) = it_prior->second; // p_wl
            b(startIndex+4) = 1;       

            // Fill out H
            startIndex = H.rows();
            H.conservativeResize(startIndex+3, dimP);
            H.block(startIndex,0,3,dimP) = Eigen::MatrixXd::Zero(3,dimP);
            H.block(startIndex,0,3,3) = skew(it_prior->second); // skew(p_wl)
            H.block(startIndex,6,3,3) = -Eigen::Matrix3d::Identity(); // -I

            // Fill out N
            startIndex = N.rows();
            N.conservativeResize(startIndex+3, startIndex+3);
            N.block(startIndex,0,3,startIndex) = Eigen::MatrixXd::Zero(3,startIndex);
            N.block(0,startIndex,startIndex,3) = Eigen::MatrixXd::Zero(startIndex,3);
            N.block(startIndex,startIndex,3,3) = R * noise_params_.getLandmarkCov() * R.transpose();

            // Fill out PI      
            startIndex = PI.rows();
            int startIndex2 = PI.cols();
            PI.conservativeResize(startIndex+3, startIndex2+dimX);
            PI.block(startIndex,0,3,startIndex2) = Eigen::MatrixXd::Zero(3,startIndex2);
            PI.block(0,startIndex2,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
            PI.block(startIndex,startIndex2,3,dimX) = Eigen::MatrixXd::Zero(3,dimX);
            PI.block(startIndex,startIndex2,3,3) = Eigen::Matrix3d::Identity();

        } else if (it_estimated!=estimated_landmarks_.end()) {;
            // Found in estimated landmark set
            int dimX = state_.dimX();
            int dimP = state_.dimP();
            int startIndex;

            // Fill out Y
            startIndex = Y.rows();
            Y.conservativeResize(startIndex+dimX, Eigen::NoChange);
            Y.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
            Y.segment(startIndex,3) = it->second; // p_bl
            Y(startIndex+4) = 1; 
            Y(startIndex+it_estimated->second) = -1;       

            // Fill out b
            startIndex = b.rows();
            b.conservativeResize(startIndex+dimX, Eigen::NoChange);
            b.segment(startIndex,dimX) = Eigen::VectorXd::Zero(dimX);
            b(startIndex+4) = 1;       
            b(startIndex+it_estimated->second) = -1;       

            // Fill out H
            startIndex = H.rows();
            H.conservativeResize(startIndex+3, dimP);
            H.block(startIndex,0,3,dimP) = Eigen::MatrixXd::Zero(3,dimP);
            H.block(startIndex,6,3,3) = -Eigen::Matrix3d::Identity(); // -I
            H.block(startIndex,3*it_estimated->second-6,3,3) = Eigen::Matrix3d::Identity(); // I

            // Fill out N
            startIndex = N.rows();
            N.conservativeResize(startIndex+3, startIndex+3);
            N.block(startIndex,0,3,startIndex) = Eigen::MatrixXd::Zero(3,startIndex);
            N.block(0,startIndex,startIndex,3) = Eigen::MatrixXd::Zero(startIndex,3);
            N.block(startIndex,startIndex,3,3) = R * noise_params_.getLandmarkCov() * R.transpose();

            // Fill out PI      
            startIndex = PI.rows();
            int startIndex2 = PI.cols();
            PI.conservativeResize(startIndex+3, startIndex2+dimX);
            PI.block(startIndex,0,3,startIndex2) = Eigen::MatrixXd::Zero(3,startIndex2);
            PI.block(0,startIndex2,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
            PI.block(startIndex,startIndex2,3,dimX) = Eigen::MatrixXd::Zero(3,dimX);
            PI.block(startIndex,startIndex2,3,3) = Eigen::Matrix3d::Identity();


        } else {
            // First time landmark as been detected (add to list for later state augmentation)
            new_landmarks.push_back(*it);
        }
    }

    // Correct state using stacked observation
    Observation obs(Y,b,H,N,PI);
    if (!obs.empty()) {
        //cout << obs << endl;
        this->Correct(obs);
    }

    // Augment state with newly detected landmarks
    if (new_landmarks.size() > 0) {
        Eigen::MatrixXd X_aug = state_.getX(); 
        Eigen::MatrixXd P_aug = state_.getP();
        Eigen::Vector3d p = state_.getPosition();
        for (auto it=new_landmarks.begin(); it!=new_landmarks.end(); ++it) {
            // Initialize new landmark mean
            int startIndex = X_aug.rows();
            X_aug.conservativeResize(startIndex+1, startIndex+1);
            X_aug.block(startIndex,0,1,startIndex) = Eigen::MatrixXd::Zero(1,startIndex);
            X_aug.block(0,startIndex,startIndex,1) = Eigen::MatrixXd::Zero(startIndex,1);
            X_aug(startIndex, startIndex) = 1;
            X_aug.block(0,startIndex,3,1) = p + R*it->second;

            // Initialize new landmark covariance - TODO:speed up
            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state_.dimP()+3,state_.dimP()); 
            F.block(0,0,state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()); // for old X
            F.block(state_.dimP()-state_.dimTheta(),6,3,3) = Eigen::Matrix3d::Identity(); // for new landmark
            F.block(state_.dimP()-state_.dimTheta()+3,state_.dimP()-state_.dimTheta(),state_.dimTheta(),state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimTheta(),state_.dimTheta()); // for theta
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(F.rows(),3);
            G.block(G.rows()-state_.dimTheta()-3,0,3,3) = R;
            P_aug = (F*P_aug*F.transpose() + G*noise_params_.getLandmarkCov()*G.transpose()).eval();

            // Add to list of estimated landmarks
            estimated_landmarks_.insert(pair<int,int> (it->first, startIndex));
        }
        state_.setX(X_aug);
        state_.setP(P_aug);
    }

    return;    
}

} // end inekf namespace