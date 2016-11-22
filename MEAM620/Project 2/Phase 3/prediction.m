function [ mu_bar, Sigma_bar, X_bar ] = prediction(F, mu, Sigma, U, Q, X, vic, dt)
%PREDICTION Summary of this function goes here
%   Detailed explanation goes here
    quat = euler_to_quat(vic(4)*dt, vic(5)*dt, vic(6)*dt);
    %%err = (quat_multiply([1 mu(4:6)'], quat))';
    mu_bar = F*mu; %+[vic(1:3,:)*dt; err(2:4); zeros(3,1)];
    Sigma_bar =F*Sigma*F' + U*Q*U';
    
    X_bar = [X(1:3)+vic(1:3,:)*dt; (quat_multiply(X(4:7)', quat))'];
             
end

