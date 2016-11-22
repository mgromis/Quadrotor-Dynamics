function [X, Z] = eskf1(sensor, vic, K_camera,XYZ, yaw, tags,splits)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
 
persistent prev_time dt previous_mu previous_Sigma nom_state R F U...
    mu_bar Sigma_bar X_bar Q
Z = [];
X = zeros(7,1);
sgm = 1000;

R_noise  = eye(7)*sgm^2;   
alpha = .8;


if( isempty(prev_time)) %initialization
    %[pos, rot] = estimate_pose(sensor,K_camera,XYZ, yaw, tags,splits);
    R = zeros(3);
    quat = mat_to_quat(R);
    nom_state = [zeros(3,1); quat];
    prev_time = vic.t;
    dt = 0.01;
    X = nom_state;
    previous_mu = zeros(6,1);
    previous_Sigma = zeros(6);
    return
    
    
end


if( ~isempty(vic.t) && ~vic.t==0) %update time

dt = dt*alpha+(1-alpha)*(vic.t-prev_time);
Q  = sgm^2*eye(6);

end



%prediction step
 %1. updating error state for prediction step  
 % u_bar = F*u
 %Sigma_bar = F*sigma*F + U*Q*U'
 %2. Update nominal state
 % p_bar = p+v_m*dt
 %q_bar = qxq{(w_m-b_g)*dt}
 %b_g_bar = b_g


 
if( ~isempty(prev_time) &&~isempty(vic.vel)) %perform prediction
    
    
    F = eye(6);
        
    U = [[-eye(3) zeros(3,3)]
        [zeros(3,3) -R]];
        
    
    [ mu_bar, Sigma_bar, X_bar ] = prediction(F, previous_mu, previous_Sigma, U, Q, nom_state, vic.vel, dt);
    previous_mu = mu_bar;
    previous_Sigma = Sigma_bar;
    nom_state = X_bar;
    prev_time = vic.t;
    %X_true_bar = X_bar;%[X(1:3)+ mu_bar(1:3); (quat_multiply(X(4:7)', [1 mu_bar(4:6)']))'; zeros(3,1)];
    
    X = nom_state; %X_true_bar; %add on error state
    R = quat_to_mat(X(4:7));
end


if(~isempty(prev_time) && ~isempty([sensor.id])) %perform update
    
    [pos, R] = estimate_pose(sensor,K_camera,XYZ, yaw, tags,splits);
    q = mat_to_quat(R);
    
    %q = [1 previous_mu(4:6)'];
    Q_dtheta = .5*[[-q(2) -q(3) -q(4)];
            [q(1) q(4) -q(3)];
            [-q(4) q(1) q(2)];
            [q(3) -q(2) q(1)]];
        
    left = eye(7);%[[eye(3) zeros(3,4) zeros(3,3)];
          %[zeros(4,3) eye(4,4) zeros(4,3)]]; 
    right = [[eye(3) zeros(3,3)]
             [zeros(4,3) Q_dtheta]];%[[eye(3) zeros(3) zeros(3)];
             %[zeros(4,3) Q_dtheta zeros(4,3)];
             %[zeros(3) zeros(3) eye(3)]];
    C = left*right;     
    z = [pos; q];
    W = .1*eye(7);
     [ mu, Sigma, X ] = update(C, mu_bar,Sigma_bar, X_bar, W, R_noise, z,R);
     
      previous_mu = mu;
      previous_Sigma = Sigma;
      nom_state = X;
     prev_time = sensor.t;
end
 


end





