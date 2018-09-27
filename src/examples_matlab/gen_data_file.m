%% Generate text file with IMU and landmark measurements
angular_velocity = w_noisy;
linear_acceleration = a_noisy;
landmark_measurements = l_noisy;
load('data/measurements/contact.mat')
load('data/measurements/encoders.mat')

%% Write data to file
fileID = fopen('imu_landmark_measurements.txt','w');
t = angular_velocity.time;
N = length(t);
for i=10:N
    %IMU 0 0 0 0 0 0 9.81
    w = angular_velocity.signals.values(i,:);
    a = linear_acceleration.signals.values(i,:);
    formatSpec = 'IMU %f %f %f %f %f %f %f\n';
    fprintf(fileID,formatSpec,t(i),w(1),w(2),w(3),a(1),a(2),a(3));
    
    %LANDMARK ID 0 0 0 
    l = landmark_measurements.signals.values(:,:,i);
    formatSpec = 'LANDMARK %f';
    str = {};
    for j=1:size(l,2)
        if ~isnan(l(1,j))
            formatSpec = [formatSpec, ' %f %f %f %f'];
            str = horzcat(str,{l(1,j), l(2,j), l(3,j), l(4,j)});
        end
    end
    if ~isempty(str)
        fprintf(fileID,[formatSpec,'\n'],t(i),str{:});
    end
end
fclose(fileID);

%% Write data to file
Cov_e = deg2rad(0.5)^2*eye(14);
fileID = fopen('imu_kinematic_measurements.txt','w');
t = angular_velocity.time;
N = length(t);
for i=10:N
    % IMU t 0 0 0 0 0 9.81
    w = angular_velocity.signals.values(i,:);
    a = linear_acceleration.signals.values(i,:);
    formatSpec = 'IMU %f %f %f %f %f %f %f\n';
    fprintf(fileID,formatSpec,t(i),w(1),w(2),w(3),a(1),a(2),a(3));
    
    
    % CONTACT t ID 0 
    c = contact.Data(i,:)';
    if c(1) < 0.9
        c(1) = 0;
    else
        c(1) = 1;
    end
    if c(2) < 0.9
        c(2) = 0;
    else
        c(2) = 1;
    end
    formatSpec = 'CONTACT %f %f %f %f %f\n'; 
    fprintf(fileID,formatSpec,t(i),0,c(1),1,c(2));

    % KINEMATICS t ID H Cov
    e = encoders.Data(i,:)';
    q_L = dcm2quat(R_VectorNav_to_LeftToeBottom(e)');
    p_L = p_VectorNav_to_LeftToeBottom(e);
    J_L = [zeros(3,14); J_VectorNav_to_LeftToeBottom(e)];
    Cov_L = reshape((J_L*Cov_e*J_L')',1,[]);
    q_R = dcm2quat(R_VectorNav_to_RightToeBottom(e)');
    p_R = p_VectorNav_to_RightToeBottom(e);
    J_R = [zeros(3,14); J_VectorNav_to_RightToeBottom(e)];
    Cov_R = reshape((J_R*Cov_e*J_R')',1,[]);
%     keyboard
    formatSpec = 'KINEMATIC %f'; % t
    str = {};
    formatSpec = [formatSpec, ' %f']; %ID
    str = horzcat(str,{0});
    for j=1:4
        formatSpec = [formatSpec, ' %f']; % q
        str = horzcat(str,{q_L(j)});
    end
    for j=1:3
        formatSpec = [formatSpec, ' %f']; % p
        str = horzcat(str,{p_L(j)});
    end
    for j=1:36
        formatSpec = [formatSpec, ' %f']; % Cov
        str = horzcat(str,{Cov_L(j)});
    end
    formatSpec = [formatSpec, ' %f']; %ID
    str = horzcat(str,{1});
    for j=1:4
        formatSpec = [formatSpec, ' %f']; % q
        str = horzcat(str,{q_R(j)});
    end
    for j=1:3
        formatSpec = [formatSpec, ' %f']; % p
        str = horzcat(str,{p_R(j)});
    end
    for j=1:36
        formatSpec = [formatSpec, ' %f']; % Cov
        str = horzcat(str,{Cov_R(j)});
    end
    formatSpec = [formatSpec, '\n'];
    fprintf(fileID,formatSpec,t(i),str{:});
end
fclose(fileID);

%% Write data to file
Cov_e = deg2rad(0.5)^2*eye(14);
fileID = fopen('imu_landmark_kinematic_measurements.txt','w');
t = angular_velocity.time;
N = length(t);
for i=10:N
    % IMU t 0 0 0 0 0 9.81
    w = angular_velocity.signals.values(i,:);
    a = linear_acceleration.signals.values(i,:);
    formatSpec = 'IMU %f %f %f %f %f %f %f\n';
    fprintf(fileID,formatSpec,t(i),w(1),w(2),w(3),a(1),a(2),a(3));
    
    %LANDMARK ID 0 0 0 
    l = landmark_measurements.signals.values(:,:,i);
    formatSpec = 'LANDMARK %f';
    str = {};
    for j=1:size(l,2)
        if ~isnan(l(1,j))
            formatSpec = [formatSpec, ' %f %f %f %f'];
            str = horzcat(str,{l(1,j), l(2,j), l(3,j), l(4,j)});
        end
    end
    if ~isempty(str)
        fprintf(fileID,[formatSpec,'\n'],t(i),str{:});
    end
    
    % CONTACT t ID 0 
    c = contact.Data(i,:)';
    if c(1) < 0.9
        c(1) = 0;
    else
        c(1) = 1;
    end
    if c(2) < 0.9
        c(2) = 0;
    else
        c(2) = 1;
    end
    formatSpec = 'CONTACT %f %f %f %f %f\n'; 
    fprintf(fileID,formatSpec,t(i),0,c(1),1,c(2));

    % KINEMATICS t ID H Cov
    e = encoders.Data(i,:)';
    q_L = dcm2quat(R_VectorNav_to_LeftToeBottom(e)');
    p_L = p_VectorNav_to_LeftToeBottom(e);
    J_L = [zeros(3,14); J_VectorNav_to_LeftToeBottom(e)];
    Cov_L = reshape((J_L*Cov_e*J_L')',1,[]);
    q_R = dcm2quat(R_VectorNav_to_RightToeBottom(e)');
    p_R = p_VectorNav_to_RightToeBottom(e);
    J_R = [zeros(3,14); J_VectorNav_to_RightToeBottom(e)];
    Cov_R = reshape((J_R*Cov_e*J_R')',1,[]);
%     keyboard
    formatSpec = 'KINEMATIC %f'; % t
    str = {};
    formatSpec = [formatSpec, ' %f']; %ID
    str = horzcat(str,{0});
    for j=1:4
        formatSpec = [formatSpec, ' %f']; % q
        str = horzcat(str,{q_L(j)});
    end
    for j=1:3
        formatSpec = [formatSpec, ' %f']; % p
        str = horzcat(str,{p_L(j)});
    end
    for j=1:36
        formatSpec = [formatSpec, ' %f']; % Cov
        str = horzcat(str,{Cov_L(j)});
    end
    formatSpec = [formatSpec, ' %f']; %ID
    str = horzcat(str,{1});
    for j=1:4
        formatSpec = [formatSpec, ' %f']; % q
        str = horzcat(str,{q_R(j)});
    end
    for j=1:3
        formatSpec = [formatSpec, ' %f']; % p
        str = horzcat(str,{p_R(j)});
    end
    for j=1:36
        formatSpec = [formatSpec, ' %f']; % Cov
        str = horzcat(str,{Cov_R(j)});
    end
    formatSpec = [formatSpec, '\n'];
    fprintf(fileID,formatSpec,t(i),str{:});
end
fclose(fileID);

%% Generate Trajectory from IMU
imu_rate = 400;
landmark_rate = 20;
t = 0:(1/imu_rate):100;
N = length(t)-1;
angular_velocity = repmat([0.1;0.1;0.1], 1, N);
linear_acceleration = repmat([0;0;0], 1, N);
R0 = eye(3);
v0 = [0;0;0];
p0 = [0;0;0];
[R, v, p, b] = GenTrajFromIMU(t,angular_velocity,linear_acceleration, R0, v0, p0);
% Add back gravity
for i=1:N
    linear_acceleration(:,i) = linear_acceleration(:,i) + R(:,:,i)'*[0;0;9.81];
end
figure;
plot3(p(1,:), p(2,:), p(3,:))

bg = [0.1;0.2;0.3];
ba = [0.1;0.2;0.3];
for i=1:N
    angular_velocity(:,i) = angular_velocity(:,i) + mvnrnd(zeros(3,1),0.01^2*eye(3))' + bg;
    linear_acceleration(:,i) = linear_acceleration(:,i) + mvnrnd(zeros(3,1),0.1^2*eye(3))' + ba;
end

% Generate Landmark measurements
landmark_positions = [[0; 1;1;0],... % [Landmark_ID; {W}_p_{WL}]
                      [1; 1;-1;0],...
                      [2; 2;0;1]];

landmark_measurements = nan(4,size(landmark_positions,2),N);
for i=1:floor(imu_rate/landmark_rate):N
    landmark_measurements(:,:,i) = [landmark_positions(1,:);
        R(:,:,i)'*(landmark_positions(2:end,:) - p(:,i))] + [0; mvnrnd(zeros(3,1),0.1^2*eye(3))'];
end

%%
% R = eye(3);
% p = 
% b = zeros(6,1);
% v0 = [0;0;0];
% [a, w, v] = GenIMUFromTraj(t, R, p, b, v0);



%% Write data to file
fileID = fopen('sim_data_2.txt','w');
for i=1:N
    %IMU 0 0 0 0 0 0 9.81
    w = angular_velocity(:,i);
    a = linear_acceleration(:,i);
    formatSpec = 'IMU %f %f %f %f %f %f %f\n';
    fprintf(fileID,formatSpec,t(i),w(1),w(2),w(3),a(1),a(2),a(3));
    
    %LANDMARK ID 0 0 0 
    l = landmark_measurements(:,:,i);
    formatSpec = 'LANDMARK %f';
    str = {};
    for j=1:size(l,2)
        if ~isnan(l(1,j))
            formatSpec = [formatSpec, ' %f %f %f %f'];
            str = horzcat(str,{l(1,j), l(2,j), l(3,j), l(4,j)});
        end
    end
    if ~isempty(str)
        fprintf(fileID,[formatSpec,'\n'],t(i),str{:});
    end
end
fclose(fileID);

