function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

T = 15;
pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;


t1 =0;
t2=0;
p1 = [0 0 0];
v1=[0 0 0];
v2=[0 0 0];
a1=[0 0 0];
a2=[0 0 0];

if t<T/4
    t1 = 0;
    t2 = T/4;
    p1 = [0 0 0];
    p2 = [1/4 2^.5 2^.5];
else if t<T/2
    t1 = T/4;
    t2 = T/2;
    p1 = [1/4 2^.5 2^.5];
    p2 =  [.5 0 2*2^.5];
    else if t< 3*T/4
    t1 = T/2;
    t2 = 3*T/4;
    p1 = [.5 0 2*2^.5];
    p2 = [3/4 -2^.5 2^.5];
        else
          t1 = 3*T/4;
    t2 = T;
    p1 = [3/4 -2^.5 2^.5]; 
    p2 = [1 0 0];
        end
    end
end

            
traj = zeros(3, 6);
A = [1 t1 t1^2 t1^3 t1^4 t1^5;
    0 1 2*t1 3*t1^2 4*t1^3 5*t1^4;
    0 0 2 6*t1 12*t1^2 20*t1^3;
    1 t2 t2^2 t2^3 t2^4 t2^5;
    0 1 2*t2 3*t2^2 4*t2^3 5*t2^4;
    0 0 2 6*t2 12*t2^2 20*t2^3];
    

for k = 1:3
    c = [p1(k), v1(k), a1(k), p2(k), v2(k), a2(k)]';
    traj(k,:) = A\c;

end

if t<T/4


pos = [traj(1,1)+traj(1,2)*t+traj(1,3)*t^2+traj(1,4)*t^3+traj(1,5)*t^4+traj(1,6)*t^5;...
    traj(2,1)+traj(2,2)*t+traj(2,3)*t^2+traj(2,4)*t^3+traj(2,5)*t^4+traj(2,6)*t^5;...
    traj(3,1)+traj(3,2)*t+traj(3,3)*t^2+traj(3,4)*t^3+traj(3,5)*t^4+traj(3,6)*t^5];
vel = [traj(1,2)+2*traj(1,3)*t+3*traj(1,4)*t^2+4*traj(1,5)*t^3+5*traj(1,6)*t^4;
        traj(2,2)+2*traj(2,3)*t+3*traj(2,4)*t^2+4*traj(2,5)*t^3+5*traj(2,6)*t^4;
        traj(3,2)+2*traj(3,3)*t+3*traj(3,4)*t^2+4*traj(3,5)*t^3+5*traj(3,6)*t^4];
acc = [2*traj(1,3)+6*traj(1,4)*t+12*traj(1,5)*t^2+20*traj(1,6)*t^3;
    2*traj(2,3)+6*traj(2,4)*t+12*traj(2,5)*t^2+20*traj(2,6)*t^3;
    2*traj(3,3)+6*traj(3,4)*t+12*traj(3,5)*t^2+20*traj(3,6)*t^3];
        
yaw = 0;
yawdot = 0;
else if t<T/2

pos = [traj(1,1)+traj(1,2)*t+traj(1,3)*t^2+traj(1,4)*t^3+traj(1,5)*t^4+traj(1,6)*t^5;...
    traj(2,1)+traj(2,2)*t+traj(2,3)*t^2+traj(2,4)*t^3+traj(2,5)*t^4+traj(2,6)*t^5;...
    traj(3,1)+traj(3,2)*t+traj(3,3)*t^2+traj(3,4)*t^3+traj(3,5)*t^4+traj(3,6)*t^5];
vel = [traj(1,2)+2*traj(1,3)*t+3*traj(1,4)*t^2+4*traj(1,5)*t^3+5*traj(1,6)*t^4;
        traj(2,2)+2*traj(2,3)*t+3*traj(2,4)*t^2+4*traj(2,5)*t^3+5*traj(2,6)*t^4;
        traj(3,2)+2*traj(3,3)*t+3*traj(3,4)*t^2+4*traj(3,5)*t^3+5*traj(3,6)*t^4];
acc = [2*traj(1,3)+6*traj(1,4)*t+12*traj(1,5)*t^2+20*traj(1,6)*t^3;
    2*traj(2,3)+6*traj(2,4)*t+12*traj(2,5)*t^2+20*traj(2,6)*t^3;
    2*traj(3,3)+6*traj(3,4)*t+12*traj(3,5)*t^2+20*traj(3,6)*t^3];
yaw = 0;
yawdot = 0;
else if t<T*3/4

pos = [traj(1,1)+traj(1,2)*t+traj(1,3)*t^2+traj(1,4)*t^3+traj(1,5)*t^4+traj(1,6)*t^5;...
    traj(2,1)+traj(2,2)*t+traj(2,3)*t^2+traj(2,4)*t^3+traj(2,5)*t^4+traj(2,6)*t^5;...
    traj(3,1)+traj(3,2)*t+traj(3,3)*t^2+traj(3,4)*t^3+traj(3,5)*t^4+traj(3,6)*t^5];
vel = [traj(1,2)+2*traj(1,3)*t+3*traj(1,4)*t^2+4*traj(1,5)*t^3+5*traj(1,6)*t^4;
        traj(2,2)+2*traj(2,3)*t+3*traj(2,4)*t^2+4*traj(2,5)*t^3+5*traj(2,6)*t^4;
        traj(3,2)+2*traj(3,3)*t+3*traj(3,4)*t^2+4*traj(3,5)*t^3+5*traj(3,6)*t^4];
acc = [2*traj(1,3)+6*traj(1,4)*t+12*traj(1,5)*t^2+20*traj(1,6)*t^3;
    2*traj(2,3)+6*traj(2,4)*t+12*traj(2,5)*t^2+20*traj(2,6)*t^3;
    2*traj(3,3)+6*traj(3,4)*t+12*traj(3,5)*t^2+20*traj(3,6)*t^3];
yaw = 0;
yawdot = 0;
    else if t<T          

pos = [traj(1,1)+traj(1,2)*t+traj(1,3)*t^2+traj(1,4)*t^3+traj(1,5)*t^4+traj(1,6)*t^5;...
    traj(2,1)+traj(2,2)*t+traj(2,3)*t^2+traj(2,4)*t^3+traj(2,5)*t^4+traj(2,6)*t^5;...
    traj(3,1)+traj(3,2)*t+traj(3,3)*t^2+traj(3,4)*t^3+traj(3,5)*t^4+traj(3,6)*t^5];
vel = [traj(1,2)+2*traj(1,3)*t+3*traj(1,4)*t^2+4*traj(1,5)*t^3+5*traj(1,6)*t^4;
        traj(2,2)+2*traj(2,3)*t+3*traj(2,4)*t^2+4*traj(2,5)*t^3+5*traj(2,6)*t^4;
        traj(3,2)+2*traj(3,3)*t+3*traj(3,4)*t^2+4*traj(3,5)*t^3+5*traj(3,6)*t^4];
acc = [2*traj(1,3)+6*traj(1,4)*t+12*traj(1,5)*t^2+20*traj(1,6)*t^3;
    2*traj(2,3)+6*traj(2,4)*t+12*traj(2,5)*t^2+20*traj(2,6)*t^3;
    2*traj(3,3)+6*traj(3,4)*t+12*traj(3,5)*t^2+20*traj(3,6)*t^3];
yaw = 0;
yawdot = 0;
        else
            pos = [1; 0; 0];
            vel = [0; 0; 0];
            acc = [0; 0; 0];
            yaw = 0;
            yawdot = 0;
            end
        end
    end
end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
