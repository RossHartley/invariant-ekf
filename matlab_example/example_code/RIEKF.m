%----------------------------------------------------------------------------
%  Copyright 2018, Ross Hartley
%  All Rights Reserved
%  See LICENSE for the license information
%---------------------------------------------------------------------------- 

%   @file   RIEKF.m
%   @author Ross Hartley
%   @brief  Matlab system object to implement contact-aided invariant extended Kalman filtering
%   @date   September 25, 2018
 

% State Estimator
classdef RIEKF < matlab.System & matlab.system.mixin.Propagates %#codegen
    
    %% Properties ==================================================
    
    % PUBLIC PROPERTIES
    properties
        % Enable Static Bias Initialization
        static_bias_initialization = true;
        % Enable Measurement Updates
        ekf_update_enabled = true;
        % Enable Kinematic measurements
        enable_kinematic_measurements = false;
        % Enable Landmark measurements
        enable_landmark_measurements = false;
        % Enable Static Landmarks
        enable_static_landmarks = false;
        % Gyroscope Noise std
        gyro_noise_std = 0.1*ones(3,1);
        % Gyroscope Bias Noise std
        gyro_bias_noise_std = 0.1*ones(3,1);
        % Accelerometer Noise std
        accel_noise_std = 0.1*ones(3,1);
        % Accelerometer Bias Noise std
        accel_bias_noise_std = 0.1*ones(3,1);
        % Contact Noise std
        contact_noise_std = 0.1*ones(3,1);
        % Encoder Noise std
        encoder_noise_std = 0.1*ones(14,1);
        % Landmark Noise std
        landmark_noise_std = 0.1*ones(3,1);
        % Gyroscope Bias Initial Condition 
        gyro_bias_init = zeros(3,1);
        % Accelerometer Bias Initial Condition
        accel_bias_init = zeros(3,1);
        % Prior Base Pose std
        prior_base_pose_std = 0.1*ones(6,1);
        % Prior Base Velocity std
        prior_base_velocity_std = 0.1*ones(3,1);
        % Prior Contact position std
        prior_contact_position_std = 0.1*ones(3,1);
        % Prior Gyroscope Bias std
        prior_gyro_bias_std = 0.1*ones(3,1);
        % Prior Accelerometer Bias std
        prior_accel_bias_std = 0.1*ones(3,1);
        % Prior Forward Kinematics std
        prior_forward_kinematics_std = 0.1*ones(3,1);
        % Static Landmark Positions
        landmark_positions = [0;0;0];
    end
    
    % PROTECTED PROPERTIES 
    properties (Access = protected)
    end 
    
    % PRIVATE PROPERTIES
    properties(Access = private)
        X;
        theta;
        P;
        filter_enabled;
        bias_initialized;
        ba0 = zeros(3,1);
        bg0 = zeros(3,1);
        a_init_vec;
        w_init_vec;
        imu_init_count = 1;
        w_prev;
        a_prev;
        encoders_prev;
        contact_prev;
        t_prev;
        landmark_ids;
        
        % Sensor Covariances
        Qg;    % Gyro Covariance Matrix
        Qbg;   % Gyro bias Covariance Matrix
        Qa;    % Accel Covariance Matrix
        Qba;   % Accel Bias Covariance Matrix
        Qc;    % Contact Covariance Matrix
        Qe;    % Encoder Covariance Matrix
        Ql;    % Landmark Distance Covariance Matrix
        Np;    % Prior Forward Kinematics Covariance Matrix
        P_prior;
        
    end
    
    % PRIVATE CONSTANTS
    properties(Access = private, Constant)
        % EKF Noise Parameters
        g = [0;0;-9.81]; % Gravity
        imu_init_total_count = 1000;
    end
    
    %% PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function setupImpl(obj)
            %SETUPIMPL Initialize System object.
            obj.filter_enabled = false;
            obj.bias_initialized = false;
            obj.a_init_vec = zeros(3, obj.imu_init_total_count);
            obj.w_init_vec = zeros(3, obj.imu_init_total_count);
            
            % Initialize Sensor Covariances
            obj.Qg = diag(obj.gyro_noise_std.^2);          % Gyro Covariance Matrix
            obj.Qbg = diag(obj.gyro_bias_noise_std.^2);    % Gyro bias Covariance Matrix
            obj.Qa = diag(obj.accel_noise_std.^2);         % Accel Covariance Matrix
            obj.Qba = diag(obj.accel_bias_noise_std.^2);   % Accel Bias Covariance Matrix
            obj.Qc = diag(obj.contact_noise_std.^2);       % Contact Covariance Matrix
            obj.Qe = diag(obj.encoder_noise_std.^2);       % Encoder Covariance Matrix
            obj.Ql = diag(obj.landmark_noise_std.^2);      % Landmark Covariance Matrix
            obj.Np = diag(obj.prior_forward_kinematics_std.^2); % Prior Forward Kinematics Covariance Matrix
            obj.P_prior = blkdiag(diag(obj.prior_base_pose_std(1:3).^2), ...
                                  diag(obj.prior_base_velocity_std.^2), ...
                                  diag(obj.prior_base_pose_std(4:6).^2), ...
                                  diag(obj.prior_contact_position_std.^2), ...
                                  diag(obj.prior_contact_position_std.^2), ...
                                  diag(obj.prior_gyro_bias_std.^2),...
                                  diag(obj.prior_accel_bias_std.^2));
                                         
            % Initialize bias estimates
            obj.bg0 = obj.gyro_bias_init;
            obj.ba0 = obj.accel_bias_init;
            
            % Initialze State and Covariance
            obj.X = eye(7); 
            obj.theta = zeros(6,1);
            obj.P = eye(21);
            
            % Variables to store previous measurements
            obj.w_prev = zeros(3,1);
            obj.a_prev = zeros(3,1);
            obj.encoders_prev = zeros(14,1);
            obj.contact_prev = zeros(2,1);
            obj.t_prev = 0;
            
            % Landmark 
            obj.landmark_ids = [];
                        
        end % setupImpl
        
        function [X, theta, P, enabled, landmark_ids] = stepImpl(obj, enable, t, w, a, encoders, contact, landmarks, X_init)
            % Function that creates a state vector from sensor readings.
            %
            %   Inputs:
            %       enable    - flag to enable/disable the filter
            %       t         - time
            %       w         - angular velocity, {I}_w_{WI}
            %       a         - linear acceleration, {I}_a_{WI}
            %       encoders  - joint encoder positions
            %       contact   - contact indicator
            %       landmarks - vector of landmark IDs along with distance
            %       X_init    - initial state
            %
            %   Outputs:
            %       X       - current state estimate
            %       theta   - current parameter estimates
            %       P       - current covariance estimate 
            %       enabled - flag indicating when the filter is enabled
            %
            %   Author: Ross Hartley
            %   Date:   1/19/2018
            %
            
%             %%% FILTER TEST
%             obj.X = eye(7);
%             obj.P = eye(21);
%             
%             landmarks = zeros(4,3);
%             landmarks(:,1) = [1; 1;2;3];
%             landmarks(:,2) = [2; 4;5;6];
%             landmarks(:,3) = [3; 7;8;9];
%             obj.Ql = 0.1^2*eye(3);
% %             obj.Update_StaticLandmarks(landmarks);
%             obj.Update_Landmarks(landmarks);    
% 
%             m = [1;2;3;4;5;6];
%             dt = 0.1;
%             obj.Qg = 0.01^2*eye(3);
%             obj.Qa = 0.1^2*eye(3);
%             obj.Qbg = 0.00001^2*eye(3);
%             obj.Qba = 0.0001^2*eye(3);
%             obj.Predict_State(m(1:3), m(4:6), zeros(14,1), zeros(2,1), dt);
%             obj.Predict_State(m(1:3), m(4:6), zeros(14,1), zeros(2,1), dt);
% 
% %             obj.Update_StaticLandmarks(landmarks);
%             obj.Update_Landmarks(landmarks);
%             %%%%%%
            
            
            % Initialize bias
            % (does nothing if bias is already initialized)
            obj.InitializeBias(w, a, X_init)
            
            % Initiaze filter
            % (does nothing if filter is already initialized)
            if t > 0.01 && any(contact == 1) 
                obj.InitializeFilter(enable, X_init);
            end
            
            % Only run if filter is enabled
            if obj.filter_enabled
           
                % Predict state using IMU and contact measurements     
                obj.Predict_State(obj.w_prev, obj.a_prev, obj.encoders_prev, obj.contact_prev, t - obj.t_prev);

                % Update using other measurements
                if obj.ekf_update_enabled

                    % Update state using forward kinematic measurements
                    if obj.enable_kinematic_measurements
                        obj.Update_ForwardKinematics(encoders, contact);
                    end
                    
                    % Update state using landmark position measurements
                    if obj.enable_landmark_measurements && ~isempty(landmarks)
                        if obj.enable_static_landmarks
                            obj.Update_StaticLandmarks(landmarks);
                        else
                            obj.Update_Landmarks(landmarks);
                        end
                    end
                    
                end
            
            end
            
            % Store last values
            obj.w_prev = w;
            obj.a_prev = a;
            obj.encoders_prev = encoders;
            obj.contact_prev = contact;
            obj.t_prev = t;
           
            % Output
            X = obj.X;
            theta = obj.theta;
            P = obj.P(1:21,1:21);
            enabled = double(obj.filter_enabled);
            landmark_ids = obj.landmark_ids;
             
        end % stepImpl

        % Define Outputs =====================================================
        function resetImpl(~)
            %RESETIMPL Reset System object states.
        end % resetImpl
        
        function [X, theta, P, enabled, landmark_ids] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            X = [7+10, 7+10];
            theta = [6,1];
            P = [21 + 3*10, 21 + 3*10];
            enabled = [1,1];
            landmark_ids = [1, 10];
        end % getOutputSizeImpl
        
        function [X, theta, P, enabled, landmark_ids] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            X = 'double';
            theta = 'double';
            P = 'double';
            enabled = 'double';
            landmark_ids = 'double';
        end % getOutputDataTypeImpl
        
        function [X, theta, P, enabled, landmark_ids] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            X = false;
            theta = false;
            P = false;
            enabled = false;
            landmark_ids = false;
        end % isOutputComplexImpl
        
        function [X, theta, P, enabled, landmark_ids] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variale-size output ports.
            X = false;
            theta = true;
            P = false;
            enabled = true;
            landmark_ids = false;
        end % isOutputFixedSizeImpl
        
    end
        
    %% PRIVATE METHODS =====================================================
    methods (Access = private)
        
        function [R, v, p, dR, dL, lm, bg, ba] = Separate_State(~, X, theta)
            % Separate state vector into components
            R = X(1:3,1:3);  % Orientation
            v = X(1:3,4);    % Base Velocity
            p = X(1:3,5);    % Base Position
            dR = X(1:3,6);   % Right Foot Position
            dL = X(1:3,7);   % Left Foot Position
            
            % Landmark Positions (if they exist)
            if size(X,2) > 7
                lm = X(1:3,8:end);
            else
                lm = [];
            end
            
            % Parameters
            bg = theta(1:3); % Gyroscope Bias
            ba = theta(4:6); % Accelerometer Bias
        end
        
        function [X, theta] = Construct_State(~, R, v, p, dR, dL, lm, bg, ba)
            % Construct matrix from separate states
            X = eye(7 + size(lm,2));
            X(1:7,1:7) = [R, v, p, dR, dL; 0,0,0,1,0,0,0; 0,0,0,0,1,0,0; 0,0,0,0,0,1,0; 0,0,0,0,0,0,1];
            % Add Landmarks
            if ~isempty(lm)
                X(1:3,8:end) = lm;
            end
            % Parameters
            theta = [bg; ba];
        end
        
        function [A] = skew(~, v)
            % Convert from vector to skew symmetric matrix
            A = [    0, -v(3),  v(2);
                  v(3),     0, -v(1);
                  -v(2),  v(1),   0];
        end
        
        function [dX, dtheta] = exp(obj, v)
            % Exponential map of SE_N(3)
            N = (length(v)-3)/3;
            Lg = zeros(N+3);
            Lg(1:3,:) = [obj.skew(v(1:3)), reshape(v(4:end),3,[])];
        	dX = expm(Lg);
            dtheta = v(end-5:end);
        end
        
        function AdjX = Adjoint(obj, X)
            % Adjoint of SE_N(3)         
            N = size(X,2)-3;
            R = X(1:3,1:3);
            R_cell = repmat({R}, 1, N+1); 
            AdjX = blkdiag(R_cell{:});
            for i = 1:N
                AdjX(3*i+1:3*i+3,1:3) = obj.skew(X(1:3,i+3))*R;
            end
        end
        
        function [] = InitializeBias(obj, w, a, X_init)
            % Function to initiaze IMU bias
            if obj.static_bias_initialization
                if ~obj.bias_initialized
                    if norm(a) > 0 && norm(w) > 0 % Wait for valid signal
                        if obj.imu_init_count <= obj.imu_init_total_count
                            % Accumulate vector of imu data
                            Rwi = X_init(1:3,1:3);
                            obj.a_init_vec(:,obj.imu_init_count) = Rwi'*(Rwi*a + obj.g);
                            obj.w_init_vec(:,obj.imu_init_count) = w;
                            obj.imu_init_count = obj.imu_init_count + 1;
                        else
                            % Use imu data vector to estimate initial biases
                            obj.ba0 = mean(obj.a_init_vec,2);
                            obj.bg0 = mean(obj.w_init_vec,2);
                            obj.bias_initialized = true;
                        end
                    end
                end
            else
                obj.bias_initialized = true;
            end
        end
        
        function [] = InitializeFilter(obj, enable, X_init)
            % Attempt to enable filter (successful if enable is true, and
            % at least one foot is on the ground)
            if enable && ~obj.filter_enabled
                obj.X = X_init;
                obj.theta = [obj.bg0; obj.ba0];
                obj.P = obj.P_prior;
                obj.filter_enabled = true;
            end
            
            % If filter is disabled, zero everything
            if ~enable || ~obj.filter_enabled
                obj.X = eye(7);
                obj.theta = zeros(6,1);
                obj.P = eye(21);
                obj.filter_enabled = false;
            end
        end
        
        function [] = Predict_State(obj, w, a, encoders, contact, dt)
            % Right-Invariant Extended Kalman Filter Prediction Step
           [R, v, p, dR, dL, lm, bg, ba] = obj.Separate_State(obj.X, obj.theta);
            
            % Bias corrected IMU information
            w_k = w - bg;    % {I}_w_{WI}
            a_k = a - ba;    % {I}_a_{WI}
            
            % Base Pose Dynamics
            R_pred = R * expm(obj.skew(w_k*dt));
            v_pred = v + (R*a_k + obj.g)*dt;
            p_pred = p + v*dt + 0.5*(R*a_k + obj.g)*dt^2;
            
            % Foot Position Dynamics
            dR_off = p_pred + R_pred * p_VectorNav_to_RightToeBottom(encoders); % {W}_p_{WR}
            dL_off = p_pred + R_pred * p_VectorNav_to_LeftToeBottom(encoders);  % {W}_p_{WL}
            dR_pred = contact(2)*dR + (1-contact(2))*dR_off;
            dL_pred = contact(1)*dL + (1-contact(1))*dL_off;
            
            % Landmark Dynamics
            lm_pred = lm;
            
            % Bias Dynamics
            bg_pred = bg; 
            ba_pred = ba;
                        
            
            % -- Linearized invariant error dynamics --
            
            % Base
            Fc = [       zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
                  obj.skew(obj.g), zeros(3), zeros(3), zeros(3), zeros(3);
                         zeros(3),   eye(3), zeros(3), zeros(3), zeros(3);
                         zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
                         zeros(3), zeros(3), zeros(3), zeros(3), zeros(3)];
           % Landmarks          
           Fc = blkdiag(Fc, zeros(3*length(obj.landmark_ids)));     
           % Parameters
           Fc = blkdiag(Fc, zeros(6));               
           Fc(1:15,end-5:end) = [             -R, zeros(3);
                                  -obj.skew(v)*R,       -R;
                                  -obj.skew(p)*R, zeros(3);
                                 -obj.skew(dR)*R, zeros(3);
                                 -obj.skew(dL)*R, zeros(3)];
           for i=1:length(obj.landmark_ids)   
               Fc(15+3*(i-1)+1:15+3*i,end-5:end) = [-obj.skew(lm(:,i))*R, zeros(3)];
           end
            
           
            % Discretize
            Fk = eye(size(Fc)) + Fc*dt; 
                        
            Lc = blkdiag(obj.Adjoint(obj.X), eye(6));
            hR_R = R_VectorNav_to_RightToeBottom(encoders);
            hR_L = R_VectorNav_to_LeftToeBottom(encoders);
            Q = blkdiag(obj.Qg, obj.Qa, zeros(3), hR_R*(obj.Qc+(1e4*eye(3).*(1-contact(2))))*hR_R', hR_L*(obj.Qc+(1e4*eye(3).*(1-contact(1))))*hR_L', zeros(3*length(obj.landmark_ids)), obj.Qbg, obj.Qba);             
            Qk = Fk*Lc*Q*Lc'*Fk'*dt; % Discretized
            
            % Construct predicted state
            [obj.X, obj.theta] = obj.Construct_State(R_pred, v_pred, p_pred, dR_pred, dL_pred, lm_pred, bg_pred, ba_pred);
            
            % Predict Covariance
            obj.P = Fk * obj.P * Fk' + Qk;
        end
        
        function [] = Update_State(obj, Y, b, H, N, PI)
            % Update State and Covariance from a measurement
            % Compute Kalman Gain
            S = H * obj.P * H' + N;
            K = (obj.P * H')/S;
            
            % Copy X along the diagonals if more than one measurement
            X_cell = repmat({obj.X}, 1, length(Y)/size(obj.X,1));
            Z = blkdiag(X_cell{:}) * Y - b;
            
            % Update State
            delta = K*PI*Z;
            dX = obj.exp(delta(1:end-6));
            dtheta = delta(end-5:end);
            obj.X = dX * obj.X;
            obj.theta = obj.theta + dtheta;
            
            % Update Covariance
            I = eye(size(obj.P));
            obj.P = (I - K*H)* obj.P *(I - K*H)' + K*N*K'; % Joseph update form
            
        end
        
        function [] = Update_ForwardKinematics(obj, encoders, contact)
            % Function to perform Right-Invariant EKF update from forward
            % kinematic measurements
            
            R_pred = obj.X(1:3,1:3);
            M = length(obj.landmark_ids);
            
            if contact(2) == 1 && contact(1) == 1 
                % Double Support
                s_pR = p_VectorNav_to_RightToeBottom(encoders);
                s_pL = p_VectorNav_to_LeftToeBottom(encoders);
                JR = J_VectorNav_to_RightToeBottom(encoders);
                JL = J_VectorNav_to_LeftToeBottom(encoders);
                
                % Measurement Model
                Y = [s_pR; 0; 1; -1; 0; zeros(M,1); 
                     s_pL; 0; 1; 0; -1; zeros(M,1)];
                b = zeros(size(Y));
                H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3,3*M), zeros(3,6);
                     zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3,3*M), zeros(3,6)];
                N = blkdiag(R_pred * JR * obj.Qe * JR' * R_pred' + obj.Np, ...
                            R_pred * JL * obj.Qe * JL' * R_pred' + obj.Np);  
                PI = [eye(3), zeros(3,4), zeros(3,M), zeros(3,7), zeros(3,M);
                     zeros(3,7), zeros(3,M), eye(3), zeros(3,4), zeros(3,M)];

                % Update State
                obj.Update_State(Y, b, H, N, PI);
                        
            elseif contact(2) == 1 
                % Single Support Right    
                s_pR = p_VectorNav_to_RightToeBottom(encoders);
                JR = J_VectorNav_to_RightToeBottom(encoders);
                
                % Measurement Model
                Y = [s_pR; 0; 1; -1; 0; zeros(M,1)];
                b = zeros(size(Y));
                H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3,3*M), zeros(3,6)];
                N = R_pred * JR * obj.Qe * JR' * R_pred' + obj.Np;
                PI = [eye(3), zeros(3,4), zeros(3,M)];

                % Update State
                obj.Update_State(Y, b, H, N, PI);
                
            elseif contact(1) == 1 
                % Single Support Left
                s_pL = p_VectorNav_to_LeftToeBottom(encoders);
                JL = J_VectorNav_to_LeftToeBottom(encoders);

                % Measurement Model
                Y = [s_pL; 0; 1; 0; -1; zeros(M,1)];
                b = zeros(size(Y));
                H = [zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3,3*M), zeros(3,6)];
                N = R_pred * JL * obj.Qe * JL' * R_pred' + obj.Np;
                PI = [eye(3), zeros(3,4), zeros(3,M)];

                % Update State
                obj.Update_State(Y, b, H, N, PI);
                
            end
        end
        

        function [] = Update_StaticLandmarks(obj, landmarks)
            % Function to perform Right-Invariant EKF update from static
            % landmark distance measurements
            
            R_pred = obj.X(1:3,1:3);
            
            % Stack landmark measurements
            Y = []; b = []; H = []; N = []; PI = []; 
            for i = 1:size(landmarks,2)
                % Search to see if measured landmark id is in the list of
                % static landmarks
                id = find(obj.landmark_positions(1,:) == landmarks(1,i));
                if ~isempty(id)
                    % Create measurement model
                    Y = vertcat(Y, [landmarks(2:end,i); 0; 1; 0; 0]);
                    b = vertcat(b, [obj.landmark_positions(2:end,id); 0; 1; 0; 0]);
                    H = vertcat(H, [obj.skew(obj.landmark_positions(2:end,id)), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), zeros(3)]);
                    N = blkdiag(N, R_pred * obj.Ql * R_pred');
                    PI = blkdiag(PI, [eye(3), zeros(3,4)]);
                end
            end
                 
            % Update State
            if ~isempty(Y)
                obj.Update_State(Y, b, H, N, PI);
            end
        end
        
        function [] = Update_Landmarks(obj, landmarks)
            % Function to perform Right-Invariant EKF update from estimated
            % landmark distance measurements            
            
            R_pred = obj.X(1:3,1:3);
                        
            % Stack landmark measurements
            Y = []; H = []; N = []; PI = []; new_landmarks = [];  
            for i = 1:size(landmarks,2)
                % Search to see if measured landmark id is in the list of
                % static landmarks
                id = find(obj.landmark_ids == landmarks(1,i));
                if isempty(id)
                    new_landmarks = horzcat(new_landmarks, landmarks(:,i));
                else
                    % Create measurement model
                    Y1 = [landmarks(2:end,i); 0; 1; 0; 0];
                    Y2 = zeros(length(obj.landmark_ids),1);
                    Y2(id) = -1;
                    Y = vertcat(Y, [Y1; Y2]);
                    
                    H1 = [zeros(3), zeros(3), -eye(3), zeros(3), zeros(3)];
                    H2 = zeros(3,3*length(obj.landmark_ids));
                    H2(:,3*(id-1)+1:3*(id-1)+3) = eye(3);
                    H3 = zeros(3,6);
                    H = vertcat(H, [H1, H2, H3]);
                    
                    N = blkdiag(N, R_pred * obj.Ql * R_pred');
                    PI = blkdiag(PI, [eye(3), zeros(3,4), zeros(3,length(obj.landmark_ids))]);
                end
            end
                 
            % Update State
            if ~isempty(Y)
                b = zeros(size(Y));
                obj.Update_State(Y, b, H, N, PI);
            end
            
            % Augment State for new landmarks
            if ~isempty(new_landmarks)
                [R, ~, p, ~, ~, ~, ~, ~] = obj.Separate_State(obj.X, obj.theta);
                for i = 1:size(new_landmarks,2)
                    % Initialize new landmark mean
                    obj.landmark_ids = horzcat(obj.landmark_ids, new_landmarks(1,i));
                    obj.X = blkdiag(obj.X, 1);
                    % {W}_p_{WL} = {W}_p_{WB} + R_{WB}*{B}_p_{BL}
                    obj.X(1:3,end) = p + R*new_landmarks(2:end,i);
                    
                    % Initialize new landmark covariance
                    F = eye(size(obj.P,1) - 6); % Start with I with state dim
                    F = vertcat(F, [zeros(3,6), eye(3), zeros(3,3*(size(obj.X,2)-6))]); % Add row to increase dimension
                    F = blkdiag(F, eye(6)); % Add block I for parameters
                    G = zeros(size(F,1),3);
                    G(end-8:end-6,:) = R_pred;
                    obj.P = F*obj.P*F' + G*obj.Ql*G';
                end
            end
            
        end
                  
        
    end % methods
end % classdef