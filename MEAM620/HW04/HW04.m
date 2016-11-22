% 
%   K =[[ 100     0     0];
%      [-0   100     0];
%      [0     0     1]];
%      
%    R = [[-1 0 0];[0 0 1]; [0 1 0]];
% T = [0; 10; 10];
% p1 = [0;0;0];
% p2 = [0; 10; 0];
% p3 = [10 0 0]';
% p4 = [10 10 0]';
% left = K*(R*p2+T);
% lambda = left(3);
% u = left(1)/lambda;
% v = left(2)/lambda;
% syms t;
% WRC = [[0 1 0]; [-cos(t) 0 sin(t)]; [sin(t) 0 cos(t)]];
% Wt = [0 0 2]';
% p = [0 0 5]';
% WRC_prime = diff(WRC,t);
% t = 0;
% WRC = subs(WRC);
% WRC_prime = subs(WRC_prime);
% % WRC = subs(WRC);
% % WRC*p+Wt
% syms t a B
% T = transpose([t*cos(a) 0 t*sin(a)]);
% R = [[cos(B) 0 sin(B)];
%     [0 1 0];
%     [-sin(B) 0 cos(B)]];
% T_hat = [[0 -T(3) T(2)];
%         [T(3) 0 -T(1)];
%         [-T(2) T(1) 0]];
%     
syms t
R = [[0 1 0];
    [-cos(t) 0 sin(t)];
    [sin(t) 0 cos(t)]];
T = [0 0 2]';
p = [0 0 5]';
K = [[100 0 200]; [0 100 300]; [0 0 1]];
P_c = K*transpose(R)*(p-T)
P_c_dot = diff(P_c, t);
t = 0;
subs(P_c_dot)

