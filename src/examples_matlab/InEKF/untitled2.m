%% Plot Covariances
t = P.time;
cov_phi_x = reshape(P.signals.values(1,1,:),1,[]);
cov_phi_y = reshape(P.signals.values(2,2,:),1,[]);
cov_phi_z = reshape(P.signals.values(3,3,:),1,[]);
cov_vel_x = reshape(P.signals.values(4,4,:),1,[]);
cov_vel_y = reshape(P.signals.values(5,5,:),1,[]);
cov_vel_z = reshape(P.signals.values(6,6,:),1,[]);
cov_pos_x = reshape(P.signals.values(7,7,:),1,[]);
cov_pos_y = reshape(P.signals.values(8,8,:),1,[]);
cov_pos_z = reshape(P.signals.values(9,9,:),1,[]);

figure(1)
subplot(3,1,1)
plot(t, sqrt(cov_phi_x))
subplot(3,1,2)
plot(t, sqrt(cov_phi_y))
subplot(3,1,3)
plot(t, sqrt(cov_phi_z))

figure(2)
subplot(3,1,1)
plot(t, sqrt(cov_vel_x))
subplot(3,1,2)
plot(t, sqrt(cov_vel_y))
subplot(3,1,3)
plot(t, sqrt(cov_vel_z))

figure(3)
subplot(3,1,1)
plot(t, sqrt(cov_pos_x))
subplot(3,1,2)
plot(t, sqrt(cov_pos_y))
subplot(3,1,3)
plot(t, sqrt(cov_pos_z))