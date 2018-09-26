function [R, v, p, b] = GenTrajFromIMU(t, w, a, R0, v0, p0, b0)

assert(size(w,1) == 3);
assert(size(a,1) == 3);
assert(size(w,2) == length(t)-1);
assert(size(a,2) == length(t)-1);
N = length(t)-1;

R = zeros(3,3,N+1);
v = zeros(3,N+1);
p = zeros(3,N+1);
b = zeros(6,N+1);
% g = [0;0;-9.81]; % gravity
g = [0;0;0]; % gravity

% Set initial state
if nargin < 4
    R0 = eye(3);
    v0 = zeros(3,1);
    p0 = zeros(3,1);
    b0 = zeros(6,1);
elseif nargin < 5
    v0 = zeros(3,1);
    p0 = zeros(3,1);
    b0 = zeros(6,1);
elseif nargin < 6
    p0 = zeros(3,1);
    b0 = zeros(6,1);
elseif nargin < 7
    b0 = zeros(6,1);
end

% Construct perfect 2nd order integration data from "imu measurements"
R(:,:,1) = R0;
v(:,1) = v0;
p(:,1) = p0;
b(:,1) = b0;
for k = 1:N
    dt = t(k+1) - t(k);
    R(:,:,k+1) = R(:,:,k)*Exp((w(:,k)-b(1:3,k))*dt);
    v(:,k+1) = v(:,k) + R(:,:,k)*(a(:,k)-b(4:6,k))*dt + g*dt;
    p(:,k+1) = p(:,k) + v(:,k)*dt + 0.5*R(:,:,k)*(a(:,k)-b(4:6,k))*dt^2 + 0.5*g*dt^2;
    b(:,k+1) = b(:,k);
end

end


function [ output ] = Exp( w )
%EXP Computes the vectorized exponential map (at the identity) as defined in:
% http://ethaneade.com/lie.pdf

A = skew(w);
theta = norm(w);

if theta == 0
    output = eye(3);
else
    output = eye(3) + (sin(theta)/theta)*A + ((1-cos(theta))/(theta^2))*A^2;
end

end

function [Ax] = skew(v)
% Convert from vector to skew symmetric matrix
Ax = [    0, -v(3),  v(2);
       v(3),     0, -v(1);
      -v(2),  v(1),     0];
end