close all;
clear;
clc;

initialize;

%% Simulate the system using ODE45
[Ts, XS] = ode45(@(t,x) noisy_model(t, x, A, B, sgm), Ts, x0); % Solve ODE
XS = XS';

%% Simualate the sensors

XSMax = max(XS, [], 2);
ZS(1,:) = XS(1,2:end) + 0.05*randn(size(XS(1,2:end)))*XSMax(1);
ZS(2,:) = XS(2,2:end) + 0.1*randn(size(XS(1,2:end)))*XSMax(2);
ZS(3,:) = XS(3,2:end) + 0.15*randn(size(XS(1,2:end)))*XSMax(3);
ZS(4,:) = XS(4,2:end) + 0.2*randn(size(XS(1,2:end)))*XSMax(4);

%% Filter
MUs = zeros(4,1001);
MUs(:,1) = mu0;
sigma = P0;
t = 0;
for i =2:size(MUs,2)
    [mu, s] = KFPrediction(MUs(:,i-1), sigma, dt, input_fun(t), A, B, B, Q);
    [mu, s] = KFUpdate(mu, s, dt, ZS(1,i-1), C, eye(1), R);
    MUs(:,i) = mu;
    sigma = s;
    t= t+dt;

end




%% Figures
figure; hold on; grid on;
plot(Ts, XS(1, :), 'r');
plot(Ts, XS(2, :), 'g');
plot(Ts, XS(3, :), 'b');
plot(Ts, XS(4, :), 'c');
plot(Ts, input_fun(Ts), 'm');
h = legend('$x$', '$x^{(i)}$', '$x^{(ii)}$', '$x^{(iii)}$', 'u', 'Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Model Simulation');


figure; hold on; grid on;
plot(Ts, XS(1, :), Ts(2:end), ZS(1, :), Ts(1:end)', MUs(1,:), 'r');
h = legend('$x$', '$z$','$mu$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Position)');

figure; hold on; grid on;
plot(Ts, XS(2, :), Ts(2:end), ZS(2, :),Ts(1:end)', MUs(2,:), 'b');
h = legend('$x$', '$z$','$mu$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Velocity)');

figure; hold on; grid on;
plot(Ts, XS(3, :), Ts(2:end), ZS(3, :), Ts(1:end)', MUs(3,:), 'k');
h = legend('$x$', '$z$','$mu$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Acceleration)');

figure; hold on; grid on;
plot(Ts, XS(4, :), Ts(2:end), ZS(4, :), Ts(1:end)', MUs(4,:), 'r');
h = legend('$x$', '$z$','$mu$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Jerk)');



    


