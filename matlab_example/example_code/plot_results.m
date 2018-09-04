%% Plot Right-Invariant EKF results
load('data/ground_truth/orientation')
load('data/ground_truth/velocity')
load('data/ground_truth/position')

%% Parse Estimated State
t = X.time;
N = length(t);

R = zeros(3,3,N); 
q = zeros(3,N);
v = zeros(3,N); vb = v;
p = zeros(3,N);
dR = zeros(3,N);
dL = zeros(3,N);
lm = cell(1,10);
bg = zeros(3,N);
ba = zeros(3,N);

Rib = [1,0,0; 0,-1,0; 0,0,-1];
lm_ids = landmark_ids.signals.values(1,:,end);
for i = 1:N
    % World Frame Estimates
    R(:,:,i) = X.signals.values(1:3,1:3,i) * Rib;
    q(:,i) = Rotation_to_Euler(R(:,:,i));
    v(:,i) = X.signals.values(1:3,4,i);
    p(:,i) = X.signals.values(1:3,5,i);
    dR(:,i) = X.signals.values(1:3,6,i);
    dL(:,i) = X.signals.values(1:3,7,i);
    
    % Landmarks
    for j = 1:length(lm)
        lm{j}(:,i) = [lm_ids(j); X.signals.values(1:3,7+j,i)];
    end
    
    % Body Frame Estimates
    vb(:,i) = R(:,:,i)' * v(:,i);
    
    % Parameters
    bg(:,i) = theta.signals.values(1:3,1,i);
    ba(:,i) = theta.signals.values(4:6,1,i);
end



%% Parse True State
t_true = orientation.Time;
N = length(t_true);

R_true = zeros(3,3,N); 
q_true = zeros(3,N);
v_true = zeros(3,N); vb_true = v_true;
p_true = zeros(3,N);
dR_true = zeros(3,N);
dL_true = zeros(3,N);
lm_true = cell(1,10);
bg_true = zeros(3,N);
ba_true = zeros(3,N);

for i = 1:N
    % World Frame 
    R_true(:,:,i) = orientation.Data(:,:,i) * Rib;
    q_true(:,i) = Rotation_to_Euler(R_true(:,:,i));
    v_true(:,i) = velocity.Data(i,:)';
    p_true(:,i) = position.Data(i,:)';
    
    % Body Frame 
    vb_true(:,i) = R_true(:,:,i)' * v_true(:,i);
    
    % Parameters
    bg_true(:,i) = gyro_bias.Data(i,:)';
    ba_true(:,i) = accel_bias.Data(i,:)';
end

% Landmarks
for j = 1:length(lm_true)
    if ~isnan(lm_ids(j))
        lm_true{j} = repmat(landmark_positions(:,lm_ids(j)),1,N);
    else
        lm_true{j} = nan(3,N);
    end
end

%% Start and end indices
% [~, startIndex] = max( diff(p(3,:)) ); 
startIndex = find(enabled.signals.values == 1, 1, 'first');
startIndex = startIndex + 1;
endIndex = length(p);

[~, startIndexTrue] = min(abs(t_true - t(startIndex)));
[~, endIndexTrue] = min(abs(t_true - t(endIndex)));

% Trim Time
t = t(startIndex:endIndex);
t_true = t_true(startIndexTrue:endIndexTrue);

% Trim Estimate
R = R(:,:,startIndex:endIndex);
q = q(:,startIndex:endIndex);
v = v(:,startIndex:endIndex); 
vb = vb(:,startIndex:endIndex);
p = p(:,startIndex:endIndex);
dR = dR(:,startIndex:endIndex);
dL = dL(:,startIndex:endIndex);
for i=1:length(lm)
    lm{i} = lm{i}(:,startIndex:endIndex);
end
bg = bg(:,startIndex:endIndex);
ba = ba(:,startIndex:endIndex);

% Trim True
R_true = R_true(:,:,startIndexTrue:endIndexTrue);
q_true = q_true(:,startIndexTrue:endIndexTrue);
v_true = v_true(:,startIndexTrue:endIndexTrue); 
vb_true = vb_true(:,startIndexTrue:endIndexTrue);
p_true = p_true(:,startIndexTrue:endIndexTrue);
dR_true = dR_true(:,startIndexTrue:endIndexTrue);
dL_true = dL_true(:,startIndexTrue:endIndexTrue);
for i=1:length(lm_true)
    lm_true{i} = lm_true{i}(:,startIndexTrue:endIndexTrue);
end
bg_true = bg_true(:,startIndexTrue:endIndexTrue);
ba_true = ba_true(:,startIndexTrue:endIndexTrue);

%% Align starting absolute position and yaw
% if ~enable_static_landmarks
%     HW1B = [R(:,:,1), p(:,1); zeros(1,3), 1];
%     HW2B = [R_true(:,:,1), p_true(:,1); zeros(1,3), 1];
%     HW2W1 = HW2B/HW1B;
%     for i = 1:length(t)
%         % World Frame
%         R(:,:,i) = HW2W1(1:3,1:3) * R(:,:,i);
%         q(:,i) = Rotation_to_Euler(R(:,:,i));
%         v(:,i) = HW2W1(1:3,1:3) * v(:,i);
%         p(:,i) = HW2W1(1:3,end) + HW2W1(1:3,1:3) * p(:,i);
%         
%         % Landmarks
%         for j = 1:length(lm)
%             lm{j}(2:4,i) = HW2W1(1:3,end) + HW2W1(1:3,1:3) * lm{j}(2:4,i);
%         end
%         
%     end
% end

%% Initialize Plots
fignum = 1;
EstimateLine = 'r-';
TrueLine = 'k--';
LineWidth = 2;

%% Orientation Plot
figure(fignum)
fignum = fignum + 1;

subplot(3,1,1)
grid on; hold on;
plot(t, rad2deg(q(1,:)), EstimateLine, 'LineWidth', LineWidth)
plot(t, rad2deg(q_true(1,:)), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Orientation - Yaw')
ylabel('degrees')

subplot(3,1,2)
grid on; hold on;
plot(t, rad2deg(q(2,:)), EstimateLine, 'LineWidth', LineWidth)
plot(t, rad2deg(q_true(2,:)), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Orientation - Pitch')
ylabel('degrees')

subplot(3,1,3)
grid on; hold on;
plot(t, rad2deg(q(3,:)), EstimateLine, 'LineWidth', LineWidth)
plot(t, rad2deg(q_true(3,:)), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Orientation - Roll')
ylabel('degrees')
xlabel('time (sec)')

%% Velocity Plot
figure(fignum)
fignum = fignum + 1;

subplot(3,1,1)
grid on; hold on;
plot(t, vb(1,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, vb_true(1,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Velocity (Body) - x')
ylabel('m/s')

subplot(3,1,2)
grid on; hold on;
plot(t, vb(2,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, vb_true(2,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Velocity (Body) - y')
ylabel('m/s')

subplot(3,1,3)
grid on; hold on;
plot(t, vb(3,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, vb_true(3,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Velocity (Body) - z')
ylabel('m/s')
xlabel('time (sec)')

%% Position Plot
figure(fignum)
fignum = fignum + 1;

subplot(3,1,1)
grid on; hold on;
plot(t, p(1,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, p_true(1,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Position (World) - x')
ylabel('m')

subplot(3,1,2)
grid on; hold on;
plot(t, p(2,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, p_true(2,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Position (World) - y')
ylabel('m')

subplot(3,1,3)
grid on; hold on;
plot(t, p(3,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, p_true(3,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Position (World) - z')
ylabel('m')
xlabel('time (sec)')

%% Landmark Plot
% for i = 1:length(lm)
%     if all(all(isnan(lm{i})))
%         continue;
%     end
%     figure(fignum)
%     fignum = fignum + 1;
%     
%     subplot(3,1,1)
%     grid on; hold on;
%     plot(t, lm{i}(2,:), EstimateLine, 'LineWidth', LineWidth)
%     plot(t, lm_true{i}(2,:), TrueLine, 'LineWidth', LineWidth)
%     legend('Estimate', 'True')
%     title(['Landmark ', num2str(lm_true{i}(1)),' (World) - x'])
%     ylabel('m')
%     
%     subplot(3,1,2)
%     grid on; hold on;
%     plot(t, lm{i}(3,:), EstimateLine, 'LineWidth', LineWidth)
%     plot(t, lm_true{i}(3,:), TrueLine, 'LineWidth', LineWidth)
%     legend('Estimate', 'True')
%     title(['Landmark ', num2str(lm_true{i}(1)),' (World) - y'])
%     ylabel('m')
%     
%     subplot(3,1,3)
%     grid on; hold on;
%     plot(t, lm{i}(4,:), EstimateLine, 'LineWidth', LineWidth)
%     plot(t, lm_true{i}(4,:), TrueLine, 'LineWidth', LineWidth)
%     legend('Estimate', 'True')
%     title(['Landmark ', num2str(lm_true{i}(1)),' (World) - z'])
%     ylabel('m')
%     xlabel('time (sec)')
% end



%% Gyroscope Bias Plot
figure(fignum)
fignum = fignum + 1;

subplot(3,1,1)
grid on; hold on;
plot(t, bg(1,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, bg_true(1,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Gyroscope Bias - x')
ylabel('rad/sec')

subplot(3,1,2)
grid on; hold on;
plot(t, bg(2,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, bg_true(2,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Gyroscope Bias - y')
ylabel('rad/sec')

subplot(3,1,3)
grid on; hold on;
plot(t, bg(3,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, bg_true(3,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Gyroscope Bias - z')
ylabel('rad/sec')
xlabel('time (sec)')

%% Accelerometer Bias Plot
figure(fignum)
fignum = fignum + 1;

subplot(3,1,1)
grid on; hold on;
plot(t, ba(1,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, ba_true(1,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Accelerometer Bias - x')
ylabel('m/s^2')

subplot(3,1,2)
grid on; hold on;
plot(t, ba(2,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, ba_true(2,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Accelerometer Bias - y')
ylabel('m/s^2')

subplot(3,1,3)
grid on; hold on;
plot(t, ba(3,:), EstimateLine, 'LineWidth', LineWidth)
plot(t, ba_true(3,:), TrueLine, 'LineWidth', LineWidth)
legend('Estimate', 'True')
title('Accelerometer Bias - z')
ylabel('m/s^2')
xlabel('time (sec)')





