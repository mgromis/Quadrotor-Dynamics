function H = data2transformation(x, y, z, phi, theta, psi)
% You may not use any built-in or downloaded functions dealing with
% rotation matrices, homogeneous transformations, Euler angles,
% roll/pitch/yaw angles, or related topics.  Instead, you must type all
% your calculations yourself, usin only low-level functions such as
% sin, cos, and vector/matrix math. 

Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R = Rx*Ry*Rz;
T = [[R; [0 0 0]] [x y z 1]'];
H = T;
