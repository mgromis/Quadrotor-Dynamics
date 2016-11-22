function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

T = 20;

if t > T
    t = T;
end

if t <= (T/4)
    y = sqrt(2) * (t / (T/4));
    z = sqrt(2) * (t / (T/4));
    t0 = 0;
    tf = (T/4);
    p0 = [0, 0, 0];
    pf = [0.25, sqrt(2), sqrt(2)];
elseif t <= (T/2)
    y = sqrt(2) - ((t-(T/4)) / (T/4))*sqrt(2);
    z = 2*sqrt(2) * (t / (T/2));
    t0 = (T/4);
    tf = (T/2);
    p0 = [0.25, sqrt(2), sqrt(2)];
    pf = [0.5, 0, 2*sqrt(2)];
elseif t <= (3*T/4)
    y = -sqrt(2)*((t-(T/2)) / (T/4));
    z = 2*sqrt(2) - ((t-(T/2)) / (T/4))*sqrt(2);
    t0 = (T/2);
    tf = (3*T/4);
    p0 = [0.5, 0, 2*sqrt(2)];
    pf = [0.75, -sqrt(2), sqrt(2)];
else
    y = sqrt(2) * ((t-T) / (T/4)); 
    z = -((t-T) / (T/4))*sqrt(2);
    t0 = (3*T/4);
    tf = T;
    p0 = [0.75, -sqrt(2), sqrt(2)];
    pf = [1, 0, 0];
end
x = t / T;

quintic_mat = [1    t0  t0^2     t0^3  t0^4        t0^5; 
               0     1  2*t0   3*t0^2  4*t0^3    5*t0^4;
               0     0     2     6*t0  12*t0^2  20*t0^3;
               1    tf  tf^2     tf^3  tf^4        tf^5; 
               0     1  2*tf   3*tf^2  4*tf^3    5*tf^4;
               0     0     2     6*tf  12*tf^2  20*tf^3;];


conditions = [[p0; zeros(1,3); zeros(1,3)]; [pf; zeros(1,3); zeros(1,3)]];
coeffs = quintic_mat \ conditions;
% Pull individual coefficients out.
%a0 = coeffs(1);
a1 = coeffs(2,:);
a2 = coeffs(3,:);
a3 = coeffs(4,:);
a4 = coeffs(5,:);
a5 = coeffs(6,:);

pos = [x; y; z];
vel = (a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4)';
acc = (2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3)';
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
