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

% Thrust
F    = 0;

% Moment    qd{qn}.yaw_des = 0
M    = zeros(3,1); % You should fill this in

% Controller parameters
kpz = 17.5;
kdz = 17.5*0.5;
kpy = 7;
kdy = 8;
kpx = kpy;
kdx = kdy;
kpph = 20.0;
kdph = 2.0;

% % Controller parameters
% kpz = 150;
% kdz = 150*0.5;
% kpy = 8;
% kdy = 8;
% kpx = kpy;
% kdx = kdy;
% kpph = 200;
% kdph = 20;    qd{qn}.yaw_des = 0

% Phi desired
psid = qd{qn}.yaw_des;

% Angles
phr = qd{qn}.euler(1);
thr = qd{qn}.euler(2);
psr = qd{qn}.euler(3);

% Angular velocities
p = qd{qn}.omega(1);
q = qd{qn}.omega(2);
r = qd{qn}.omega(3);

% Commanded values
% Positions
rz_acc = qd{qn}.acc_des(3)+kdz*(qd{qn}.vel_des(3)-qd{qn}.vel(3))+...
            kpz*(qd{qn}.pos_des(3)-qd{qn}.pos(3));
% ry_acc = params.grav*(thr*sin(psr)-phr*cos(psr));
ry_acc = qd{qn}.acc_des(2)+kdy*(qd{qn}.vel_des(2)-qd{qn}.vel(2))+...
           kpy*(qd{qn}.pos_des(2)-qd{qn}.pos(2));
% rx_acc = params.grav*(thr*cos(psr)+phr*sin(psr));
rx_acc = qd{qn}.acc_des(1)+kdx*(qd{qn}.vel_des(1)-qd{qn}.vel(1))+...
           kpx*(qd{qn}.pos_des(1)-qd{qn}.pos(1));

valz = 5;
valy = 5;
valx = 4;
if abs(rz_acc)>=valz
    rz_acc = sign(rz_acc)*valz;
end
if abs(ry_acc)>=valy
    ry_acc = sign(ry_acc)*valy;
end
if abs(rx_acc)>=valx
    rx_acc = sign(rx_acc)*valx;
end
% Commanded angles
phc = (1/params.grav)*(rx_acc*sin(psid)-ry_acc*cos(psid));
thc = (1/params.grav)*(rx_acc*cos(psid)+ry_acc*sin(psid));
psc = qd{qn}.yaw_des;

if phc > params.maxangle
    phc = params.maxangle;
end
if thc > params.maxangle
    thc = params.maxangle;
end
if psc > params.maxangle
    psc = params.maxangle;
end

% Forces and Moments
F = params.mass*(params.grav+rz_acc);
M = params.I*[kdph*(-p)+kpph*(phc-phr);
              kdph*(-q)+kpph*(thc-thr);
              kdph*(-r)+kpph*(psc-psr)];
thrust = F/ params.grav * 1000;

% =================== Your code ends here ===================
 
% Output trpy and drpy as in hardware
trpy = [thrust , phc, thc, psc];
drpy = [0, 0,       0,         0];

end

