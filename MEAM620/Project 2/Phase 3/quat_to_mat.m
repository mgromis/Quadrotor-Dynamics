function [ R ] = quat_to_mat( q )
%QUAT_TO_MAT Summary of this function goes here
%   Detailed explanation goes here
x = q(2);
y = q(3);
z = q(4);
w = q(1);

R = [[1-2*y^2-2*z^2  2*x*y+2*w*z 2*x*z-2*w*y];
    [2*x*y-2*w*z 1-2*x^2-2*z^2 2*y*z+2*w*x];
    [2*x*z+2*w*y 2*y*z-2*w*x 1-2*x^2-2*y^2]];

end

