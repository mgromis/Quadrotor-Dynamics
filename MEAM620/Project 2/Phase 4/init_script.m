% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% eskf_map_handle = @(sensor) eskf1(sensor, your_input);
%
% We will only call eskf_map_handle in the test function.
% Note that this will only create a function handle, but not run the function

% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) eskf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) eskf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function



% Camera Matrix (zero-indexed):
K_camera = [314.1779 0         199.4848; ...
0         314.2218  113.7838; ...
0         0         1];

% Camera-IMU Calibration (see attached images for details):
XYZ = [-0.04, 0.0, -0.03];
yaw = pi/4;

% Tag ids:
tags = [0, 12, 24, 36, 48, 60, 72, 84,  96;
 1, 13, 25, 37, 49, 61, 73, 85,  97;
 2, 14, 26, 38, 50, 62, 74, 86,  98;
 3, 15, 27, 39, 51, 63, 75, 87,  99;
 4, 16, 28, 40, 52, 64, 76, 88, 100;
 5, 17, 29, 41, 53, 65, 77, 89, 101;
 6, 18, 30, 42, 54, 66, 78, 90, 102;
 7, 19, 31, 43, 55, 67, 79, 91, 103;
 8, 20, 32, 44, 56, 68, 80, 92, 104;
 9, 21, 33, 45, 57, 69, 81, 93, 105;
10, 22, 34, 46, 58, 70, 82, 94, 106;
11, 23, 35, 47, 59, 71, 83, 95, 107];


rx = [1 0 0;
       0 cos(pi) -sin(pi);
       0 sin(pi) cos(pi)];
   rz = [cos(-yaw) -sin(-yaw) 0;
       sin(-yaw) cos(-yaw) 0;
       0 0 1];    
f2b = rz*rx;

splits = [0 .152, .152, .178, .152, .152,.178,.152,.152];


eskf_map_handle = @(sensor) eskf_map(sensor, K_camera, XYZ, yaw, tags,splits);
