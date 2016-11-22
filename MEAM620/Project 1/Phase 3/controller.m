function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
phi_des = 0;
theta_des = 0;
psi_des = 0;


roll =qd{1}.euler(1);
pitch =qd{1}.euler(2);
yaw =qd{1}.euler(3);
phi = roll;
theta= pitch;
psi = yaw;
p = qd{1}.omega(1);
q =qd{1}.omega(2);
r =qd{1}.omega(3);

worldToBody = RPYtoRot_ZXY(roll,pitch,yaw);
bodyToWorld = worldToBody';

m = params.mass;
g = params.grav;

kp = 10;%10;
kd = 5;%5;
kp_ang = 200;%200;
kd_ang = 20;%20;



pos_des = qd{1}.pos_des;
pos_e = qd{1}.pos_des-qd{1}.pos;
vel_e = qd{1}.vel_des-qd{1}.vel;
acc = qd{1}.acc_des;
rdd = acc+kd*vel_e+kp*pos_e;
Fdes = m*g + m*(rdd(3));


% Thurst
F = Fdes;

phi_des = 1/g*(rdd(1)*sin(psi)-rdd(2)*cos(psi));
theta_des = 1/g*(rdd(1)*cos(psi)+rdd(2)*sin(psi));



M = params.I*[kp_ang*(phi_des-roll)+kd_ang*(-p); ...
    kp_ang*(theta_des-pitch)+kd_ang*(-q); kp_ang*(psi_des-yaw)+kd_ang*(-r)];

% Moment
%M    = zeros(3,1); % You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
