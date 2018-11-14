%% Generate text file with IMU and landmark measurements
load('data/ground_truth/orientation.mat')
load('data/ground_truth/position.mat')
load('data/ground_truth/velocity.mat')

%% Write data to file
fileID = fopen('sim_v1_ground_truth.txt','w');
t = orientation.time;
N = length(t);
for i=1:N
    % (t, orientation, positon, velocity)
    ypr = Rotation_to_Euler(orientation.Data(:,:,i));
    pos = position.Data(i,:);
    vel = velocity.Data(i,:);
    formatSpec = '%f %f %f %f %f %f %f %f %f %f \n';
    fprintf(fileID,formatSpec,t(i),ypr(1),ypr(2),ypr(3),...
                                   pos(1),pos(2),pos(3),...
                                   vel(1),vel(2),vel(3));
end
fclose(fileID);
