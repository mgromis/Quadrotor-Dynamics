function [ mu, Sigma, X ] = update2(C, mu_bar,Sigma_bar, X_bar, W, R_noise, z, R)
%UPDATE Summary of this function goes here
%   Detailed explanation goes here
    K = Sigma_bar*C'*(C*Sigma_bar*C' + W*R_noise*W')^-1;
    mu_prime = K*(z-[X_bar(1:3); X_bar(7:10)]);
    Sigma_prime = (eye(size(K,1))-K*C)*Sigma_bar*(eye(size(K,1))-K*C)'+...
        K*W*R_noise*W'*K';
   
    
    X = X_bar;
    d_theta = mu_prime(7:9);
    X(1:3) = (X(1:3) + mu_prime(1:3))';
    X(4:6) = (X(4:6) + mu_prime(4:6))';
    X(7:10) = (quat_multiply(X(7:10)', [1 d_theta']))';
    X(11:16) = X(11:16)+ mu_prime(10:15);
    %isequal((quat_multiply(X_bar(4:7)', [1 d_theta']))', (quatmultiply(X_bar(4:7)', [1 d_theta']))');
    %X(8:10) = zeros(3,1);
    mu =zeros(15,1);
    hat = .5*[[0 -d_theta(3) d_theta(2)];
            [d_theta(3) 0 -d_theta(1)];
            [-d_theta(2) d_theta(1) 0]];
    H = eye(15);
    H(7:9,7:9) = H(7:9,7:9) +hat;
    %[[eye(3) zeros(3) zeros(3)];
%         [zeros(3) eye(3) zeros(3)];
%         [zeros(3) zeros(3) eye(3)]];
    
    Sigma = H*Sigma_prime*H';
    
    
    
end

