function [desired_state] = circle(t, qn, initial_pos)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
persistent tt offset

r = 5;     %[m]

if nargin > 2
    tt = t;
    pos0 = initial_pos;
    offset = pos0 - [r, 0, 0];
end

yaw = 0;
yawdot = 0;

% Solve the necessary quintict equations
matrix = [tt^5 tt^4 tt^3; 5*tt^4 4*tt^3 3*tt^2; 20*tt^3 12*tt^2 6*tt];
b = [2*pi 0 0]';
vec = matrix\b;
a = vec(1); b = vec(2); c = vec(3);
theta   = a*t^5+b*t^4+c*t^3;
thetad  = 5*a*t^4+4*b*t^3+3*c*t^2;
thetadd = 20*a*t^3+12*b*t^2+6*c*t;


% Equations for the trajectory of a circle
pos = [r*cos(theta); r*sin(theta); h*theta/(2*pi)] + offset;
vel = [-r*thetad*sin(theta); r*thetad*cos(theta); h*thetad/(2*pi)];
acc = [-r*thetadd*sin(theta)-r*thetad^2*cos(theta);-r*thetad^2*sin(theta)+r*thetadd*cos(theta); h*thetadd/(2*pi)];

% At the end of the cycle continue with the last known parameters to get to
% the final coordinate.
if t >= tt
    t = tt;
    
    % Resolve for theta
    theta   = a*t^5+b*t^4+c*t^3;
    thetad  = 5*a*t^4+4*b*t^3+3*c*t^2;
    thetadd = 20*a*t^3+12*b*t^2+6*c*t;

    pos = [r*cos(theta); r*sin(theta); h*theta/(2*pi)] + offset;
    vel = [-r*thetad*sin(theta); r*thetad*cos(theta); h*thetad/(2*pi)];
    acc = [-r*thetadd*sin(theta)-r*thetad^2*cos(theta);-r*thetad^2*sin(theta)+r*thetadd*cos(theta); h*thetadd/(2*pi)];
end

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
