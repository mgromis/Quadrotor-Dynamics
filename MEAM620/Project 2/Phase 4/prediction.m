function [ mu_bar, Sigma_bar, X_bar ] = prediction(F, mu, Sigma, U, Q, X, sensor, dt,R)
%PREDICTION Summary of this function goes here
%   Detailed explanation goes here
    quat = euler_to_quat((sensor.omg(1)-X(14))*dt, (sensor.omg(2)-X(15))*dt,...
        (sensor.omg(3)-X(16))*dt);
    
    mu_bar = F*mu; 
    Sigma_bar =F*Sigma*F' + U*Q*U';
    
    p = X(1:3)+X(4:6)*dt+ .5*(R*(sensor.acc-X(11:13))+[0; 0; -9.81])*dt^2;
        
    v = X(4:6)+(R*(sensor.acc-X(11:13))+[0; 0; -9.81])*dt;
    
    q = (quat_multiply(X(7:10)', quat))';
    X_bar = [p; v; q; X(11:end)];
             
end



