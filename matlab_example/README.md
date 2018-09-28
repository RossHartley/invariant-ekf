## Running the Matlab example
1. Open Matlab to the "matlab_example" folder.

2. Run the scipt "run_RIEKF_test.m". This will open and run a simulink model with the measurement data stored in the "/data" folder. After the simulation finishes, a few plots will appear analyzing the results of the state estimator.   

3. The filter parameters can be changed in the "example_code/RIEKF_InitFcn.m" script. This script is automatically executed when the simulink model is run.

### Tunable Parameters
The following parameters will affect the actual noisy measurements coming into the filter:
* `gyro_true_bias_init` - Initial gyroscope bias 
* `accel_true_bias_init` - Initial accelerometer bias 
* `gyro_true_noise_std` - Standard deviation of noise added to the gyroscope measurement 
* `gyro_true_bias_noise_std` - Standard deviation of noise added to the gyroscope bias 
* `accel_true_noise_std` - Standard deviation of noise added to the accelerometer measurement 
* `accel_true_bias_noise_std` - Standard deviation of noise added to the accelerometer bias 
* `landmark_true_noise_std` - Standard deviation of noise added to the landmark measurements 
* `landmark_positions` - List of landmark positions with an associated ID
* `landmark_measurement_frequency` - Frequency of incoming landmark measurements

The following parameters will affect how the filter is run:
* `static_bias_initialization` - Flag that enables static bias initialization, where the initial bias estimate is obtained from the first few seconds of data assuming the base pose remains fixed. Keep this flag to false for the included dataset.
* `ekf_update_enabled` - Flag that enables the update phase of the Kalman filter.
* `enable_kinematic_measurements` - Flag that enables kinematic measurements.
* `enable_landmark_measurements` - Flag that enables landmark measurements.
* `enable_static_landmarks` - Flag that enables static landmarks. If false, the landmark positions will be estimated along with the rest of the state variables.

The following parameters affect the initial condition and covariances used for the process and measurement models:
* `gyro_bias_init` - Initial gyroscope bias estimate
* `accel_bias_init` - Initial accelerometer bias estimate
* `gyro_noise_std` - Standard deviation of the gyroscope measurement noise
* `gyro_bias_noise_std` - Standard deviation of the gyroscope bias noise
* `accel_noise_std` - Standard deviation of the accelerometer measurement noise
* `accel_bias_noise_std` - Standard deviation the accelerometer bias noise
* `contact_noise_std` - Standard deviation of the contact frame velocity measurement noise
* `encoder_noise_std` - Standard deviation of the joint encoder measurement noise
* `landmark_noise_std` - Standard deviation of the landmark measurement noise

The following parameters set the initial covariance for the state estimate:
* `prior_base_pose_std` - Initial base orientation and position standard deviation
* `prior_base_velocity_std` - Initial base velocity standard deviation
* `prior_contact_position_std` - Initial contact position standard deviation
* `prior_gyro_bias_std` - Initial gyroscope bias standard deviation
* `prior_accel_bias_std` - Initial accelerometer bias standard deviation
* `prior_forward_kinematics_std` - Additional noise term that is added to increase the forward kinematics measurement covariance
