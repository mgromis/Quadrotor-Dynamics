function [mu, sigma] = KFPrediction(mu, sigma, dt, u, A, B, U, Q)
% KFPrediction computes the predicted mean and covariance using
% the dynamic model x_dot = A*x + B*u + U*n
% Inputs:
%   mu      prior mean
%   sigma 	prior covariance
%   dt      time step
%   u       control input
%   A       from the transition model
%   B       from the transition model
%   U       from the transition model
%   Q       covariance matrix of the input noise n(t)
% Outputs:
%   mu      updated mean
%   sigma 	updated covariance

mu = mu+dt*(A*mu+B*u);
Ft = (eye(size(A,1))+dt*A);
Vt = dt*U;
sigma =Ft*sigma*Ft'+ Vt*Q*Vt';


