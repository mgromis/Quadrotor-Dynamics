function [desired_state] = single_point(t, qn, qd, initial_pos, final_pos)
% CIRCLE trajectory generator for a circle


% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

persistent pos0 posf T


if nargin > 3
    pos0 = initial_pos + qd{qn}.pos;
    posf = final_pos + qd{qn}.pos;
    T = t;
    
end


if t > T
    t = T;
end

t0 = 0;
tf = T;


p0 = pos0';
pf = posf';

quintic_mat = [1    t0  t0^2     t0^3  t0^4        t0^5; 
               0     1  2*t0   3*t0^2  4*t0^3    5*t0^4;
               0     0     2     6*t0  12*t0^2  20*t0^3;
               1    tf  tf^2     tf^3  tf^4        tf^5; 
               0     1  2*tf   3*tf^2  4*tf^3    5*tf^4;
               0     0     2     6*tf  12*tf^2  20*tf^3;];


conditions = [[p0; zeros(1,3); zeros(1,3)]; [pf; zeros(1,3); zeros(1,3)]];
coeffs = quintic_mat \ conditions;
% Pull individual coefficients out.
a0 = coeffs(1,:);
a1 = coeffs(2,:);
a2 = coeffs(3,:);
a3 = coeffs(4,:);
a4 = coeffs(5,:);
a5 = coeffs(6,:);

pos = (a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5)';
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
