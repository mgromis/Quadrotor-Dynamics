function [ mu, Sigma, X ] = update(C, mu_bar,Sigma_bar, X_bar, W, R_noise, z_m, z_p, error_state_len, keyframe_state_idx, keyframe_error_idx, state_map )
%UPDATE Summary of this function goes here
%   Detailed explanation goes here


K = Sigma_bar*C'*(C*Sigma_bar*C' + W*R_noise*W')^-1;
mu_prime = K*(z_m-z_p);
Sigma_prime = (eye(size(K,1))-K*C)*Sigma_bar*(eye(size(K,1))-K*C)'+...
    K*W*R_noise*W'*K';




X = X_bar;
d_theta = mu_prime(7:9);
X(1:3) = (X(1:3) + mu_prime(1:3))';
X(4:6) = (X(4:6) + mu_prime(4:6))';
X(7:10) = (quat_multiply(X(7:10)', [1 d_theta']))';
X(11:16) = X(11:16)+ mu_prime(10:15);


h = .5*[[0 -d_theta(3) d_theta(2)];
    [d_theta(3) 0 -d_theta(1)];
    [-d_theta(2) d_theta(1) 0]];
H = eye(error_state_len);
H(7:9,7:9) = H(7:9,7:9) +h;


for j=1:size(keyframe_state_idx,1)
    s_idx = keyframe_state_idx(j);
    e_idx = keyframe_error_idx(j);
    p = X_bar(s_idx:s_idx+2)+mu_prime(e_idx:e_idx+2);
    q = [1, .5*mu_prime(e_idx+3:e_idx+5)'];
    q = q/norm(q);
    
    quat = quat_multiply(X_bar(s_idx+3:s_idx+6)',q);
    X(s_idx:s_idx+6) = [p;quat'];
    h = .5*hat(quat_to_mat(quat));
    H(e_idx+3:e_idx+5,e_idx+3:e_idx+5) = ...
    H(e_idx+3:e_idx+5,e_idx+3:e_idx+5) +h;
end




for i = 1:size(state_map,1)
    if state_map(i,1)
        s_idx = state_map(i,24);
        e_idx = state_map(i,23);
        has_dfx_dfy = state_map(i,22);
        if has_dfx_dfy
            X(s_idx:s_idx+11) = X_bar(s_idx:s_idx+11)...
                +mu_prime(e_idx:e_idx+11);
        else
            error_update = zeros(12,1);
            error_update([3,6,9,10,11,12]) = mu_prime(e_idx:e_idx+5);
            X(s_idx:s_idx+11) = X_bar(s_idx:s_idx+11) + error_update;
        end
        state_map(i,9:20) = X(s_idx:s_idx+11);
        keyframe_num = state_map(i,25);
        K_idx = keyframe_state_idx(keyframe_num);
        state_map(i,2:8) = X(K_idx:K_idx+6);
    end
       
end


mu = zeros(error_state_len,1);
Sigma = H*Sigma_prime*H';



end

