function [a, w, v] = GenIMUFromTraj(t, R, p, b, v0)

N = length(t);
w = zeros(3,N-1);
a = zeros(3,N-1);
v = zeros(3,N-1);
g = [0;0;-9.81]; % gravity

% Construct perfect "imu measurements" from 2nd order IMU model
v(:,1) = v0;
for k = 1:N-1
    dt = t(k+1) - t(k);
    w(:,k) = Log(R(:,:,k)'*R(:,:,k+1) + b(1:3,k)*dt) * (1/dt);
    a(:,k) = (2*R(:,:,k)'*(p(:,k+1) - p(:,k) - v(:,k)*dt - 0.5*g*dt^2) + b(4:6,k)*dt^2) * (1/dt^2);
    v(:,k+1) = v(:,k) + R(:,:,k)*(a(:,k)-b(4:6,k))*dt + g*dt;
end

end

function [ output ] = Log( A )
%LOG Computes the vectorized log map (at the identity) as defined in:
% http://www.roboticsproceedings.org/rss11/p06.pdf

theta = acos((trace(A)-1)/2);

tol = 1e-6;
if (theta < tol)
    output = zeros(3,1);
    return;
end

output = unskew(theta*(A-A')/(2*sin(theta)));

end

function [v] = unskew(Ax)
% Convert skew symmetric matrix to vector
v = [Ax(3,2); Ax(1,3); Ax(2,1)];
end