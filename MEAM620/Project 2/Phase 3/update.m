function [ mu, Sigma, X ] = update(C, mu_bar,Sigma_bar, X_bar, W, R_noise, z, R)
%UPDATE Summary of this function goes here
%   Detailed explanation goes here
    K = Sigma_bar*C'*(C*Sigma_bar*C' + W*R_noise*W')^-1;
    mu_prime = K*(z-X_bar);
    Sigma_prime = (eye(size(K,1))-K*C)*Sigma_bar*(eye(size(K,1))-K*C)'+...
        K*W*R_noise*W'*K';
   
    
   
    d_theta = mu_prime(4:6);
    X(1:3) = (X_bar(1:3) + mu_prime(1:3))';
    X(4:7) = (quat_multiply(X_bar(4:7)', [1 d_theta']))';
    %isequal((quat_multiply(X_bar(4:7)', [1 d_theta']))', (quatmultiply(X_bar(4:7)', [1 d_theta']))');
    %X(8:10) = zeros(3,1);
    X = X';
    mu =zeros(6,1);
    hat = .5*[[0 -d_theta(3) d_theta(2)];
            [d_theta(3) 0 -d_theta(1)];
            [-d_theta(2) d_theta(1) 0]];
    H = [[eye(3) zeros(3)];
        [ zeros(3) (eye(3)+hat)]];
    %[[eye(3) zeros(3) zeros(3)];
%         [zeros(3) eye(3) zeros(3)];
%         [zeros(3) zeros(3) eye(3)]];
    
    Sigma = H*Sigma_prime*H';
    
    
    
end

