function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables


%implemented bell curve trajectory.


T = 20;
f = .0001;
m = 2*pi/((pi^.5*erf(f^.5*(T-T/2))/(2*f^.5)-T*exp(-f*T^2/4))-pi^.5*...
    erf(-f^.5*T/2)/(2*f^.5));
c =-m*pi^.5*erf(f^.5*T/2)/(2*f^.5)+m*T*exp(-f*T^2/4)+2*pi;
w = m*(exp(-(f^.5*t-f^.5*T/2)^2)-exp(-f*T^2/4));
theta = m*(pi^.5*erf(f^.5*(t-T/2))/(2*f^.5)-t*exp(-f*T^2/4))+c;
d = f*m*(T-2*t)*exp(-.25*f*(T-2*t)^2);
pos = [5*cos(theta); 5*sin(theta); 2.5/(2*pi)*theta];
vel = [-5*(w)*sin(theta); 5*(w)*cos(theta); 2.5/(2*pi)*w];
acc = [-5*(w)^2*cos(theta); -5*(w)^2*sin(theta); 2.5/(2*pi)*d];

    yaw = 0;
    yawdot = 0;
if (t>=T) 

    pos = [5; 0; 2.5];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end


    

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
