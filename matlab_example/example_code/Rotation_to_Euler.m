function [q] = Rotation_to_Euler(R)
% Extract Euler angles in (RzRyRx) convention
% https://www.geometrictools.com/Documentation/EulerAngles.pdf
%
% Author:   Ross Hartley
% Date:     06/23/2017

% Singularity at qy = +/- pi/2
if R(3,1) < 1 
    if R(3,1) > -1 
        % Unique solution
        qx = atan2(R(3,2),R(3,3));
        qy = asin(-R(3,1));
        qz = atan2(R(2,1),R(1,1));
    else
        % Not a unique solution
        qx = 0;
        qy = pi/2;
        qz = -atan2(-R(2,3),R(2,2));
    end
else
    % Not a unique solution
    qx = 0;
    qy = -pi/2;
    qz = atan2(-R(2,3),R(2,2));
end

q = [qz; qy; qx];
end

