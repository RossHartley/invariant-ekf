%% Generate text file with IMU and landmark measurements
angular_velocity = w_noisy;
linear_acceleration = a_noisy;
landmark_measurements = l_noisy;

%% Write data to file
fileID = fopen('sim_data.txt','w');
t = angular_velocity.time;
N = length(t);
for i=1:N
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


%% Generate Trajectory from IMU
imu_rate = 200;
landmark_rate = 20;
t = 0:(1/imu_rate):100;
N = length(t)-1;
angular_velocity = repmat([0;0;0.1], 1, N);
linear_acceleration = repmat([0.005;0;9.81+0.001], 1, N);
[R, v, p, b] = GenTrajFromIMU(t,angular_velocity,linear_acceleration);

% Generate Landmark measurements
landmark_positions = [[1; 1;1;0],... % [Landmark_ID; {W}_p_{WL}]
                      [2; 1;-1;0],...
                      [3; 1;0;1]];

landmark_measurements = nan(4,size(landmark_positions,2),N);
for i=1:floor(imu_rate/landmark_rate):N
    landmark_measurements(:,:,i) = [landmark_positions(1,:);
        R(:,:,i)'*(landmark_positions(2:end,:) - p(:,i))];
end

figure;
plot3(p(1,:), p(2,:), p(3,:))

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

