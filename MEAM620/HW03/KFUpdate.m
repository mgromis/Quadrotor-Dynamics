function [mu, sigma] = KFUpdate(mu, sigma, dt, z, C, W, R)
% KFUpdate computes the updated mean and covariance using
% the observation model z(t) = C*x(t) + W*v(t)
% Inputs:
%   mu      predicted mean
%   sigma   predicted covariance
%   dt      time step
%   z       measurement
%   C       from the observation model
%   W       from the observation model
%   R       covariance matrix of the input noise v(t)
% Outputs:
%   mu      updated mean
%   sigma 	updated covariance
Kt = sigma*C'*(C*sigma*C'+W*R*W')^-1;
mu = mu+Kt*(z-C*mu);
sigma = sigma-Kt*C*sigma;