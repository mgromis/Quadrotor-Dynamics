function [X] = eskf2(sensor, K_camera,XYZ, yaw, tags,splits)
% EKF2 Extended Kalman Filter with IMU as inputs
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
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

persistent prev_time dt previous_mu previous_Sigma nom_state R F U...
    mu_bar Sigma_bar X_bar Q
X = zeros(16,1);
sgm = .1;

R_noise  = eye(7)*sgm^2;   
alpha = .8;


if( isempty(prev_time) && ~isempty(sensor)) %initialization
    [pos, rot] = estimate_pose(sensor,K_camera,XYZ, yaw, tags,splits);
    R = rot;
    quat = mat_to_quat(R);
    nom_state = [pos; zeros(3,1); quat; zeros(6,1)];
    prev_time = sensor.t;
    dt = 0.01;
    X = nom_state;
    previous_mu = zeros(15,1);
    previous_Sigma = zeros(15);
    return
    
    
end


if(isempty(prev_time))
    dt =.02 ;
else
    
    dt = dt*alpha+(1-alpha)*(sensor.t-prev_time);
end
sgm_acc = .1;
sgm_gyro = .1;
n_bias_acc = .1;
n_gyro_bias = .1;
vi = sgm_acc^2*dt^2*eye(3);
theta_i =sgm_gyro^2*dt^2*eye(3);
A_i = n_bias_acc^2*dt*eye(3);
omega_i = n_gyro_bias^2*dt*eye(3);


Q  = zeros(12);
Q(1:3,1:3) = vi;
Q(4:6,4:6) = theta_i;
Q(7:9,7:9) = A_i;
Q(10:12,10:12) = omega_i;
 




%prediction step
 %1. updating error state for prediction step  
 % u_bar = F*u
 %Sigma_bar = F*sigma*F + U*Q*U'
 %2. Update nominal state
 % p_bar = p+v_m*dt
 %q_bar = qxq{(w_m-b_g)*dt}
 %b_g_bar = b_g


 
if( ~isempty(prev_time) && sensor.is_ready) %perform prediction
    
    O = zeros(3);
    I = eye(3);
    
    Acc_mat = (sensor.acc-nom_state(11:13));
    Acc_SS = [0 -Acc_mat(3) Acc_mat(2) ; Acc_mat(3) 0 -Acc_mat(1) ;...
        -Acc_mat(2) Acc_mat(1) 0 ];
    Omg_vec = (sensor.omg-nom_state(14:16))*dt;
    omg_rot = quat_to_mat(euler_to_quat( Omg_vec(1), Omg_vec(2),Omg_vec(3)));
    omg_SS = [0 -Omg_vec(3) Omg_vec(2) ; Omg_vec(3) 0 -Omg_vec(1) ;...
        -Omg_vec(2) Omg_vec(1) 0 ];
    F = [[I I*dt O O O];
        [O I -R*Acc_SS*dt -R*dt O];
        [O O R'*omg_SS O -R*dt];
        [O O O I O];
        [O O O O I]];
    %[[I I*dt O O O];
        %[O I Acc_SS*dt -R*dt O];
        %[O O I O -R*dt];
        %[O O O I O];
        %[O O O O I]];
        
        
        
        
    U = [[O O O O];
        [R O O O];
        [O R O O ];
        [O O R O];
        [O O O O]];
        
    
    [ mu_bar, Sigma_bar, X_bar ] = prediction2(F, previous_mu, previous_Sigma, U, Q, nom_state, sensor, dt,R);
    previous_mu = mu_bar;
    previous_Sigma = Sigma_bar;
    nom_state = X_bar;
    prev_time = sensor.t;  
    X = nom_state; %X_true_bar; %add on error state
    R = quat_to_mat(X(7:10));
end


if(~isempty(prev_time) && ~isempty([sensor.id])) %perform update
    
    [pos, R] = estimate_pose(sensor,K_camera,XYZ, yaw, tags,splits);
    %[vel, ~] = estimate_vel(sensor, K_camera, XYZ, yaw, tags,splits);
    q = mat_to_quat(R);
    
    %q = [1 previous_mu(4:6)'];
    Q_dtheta = .5*[[-q(2) -q(3) -q(4)];
            [q(1) q(4) -q(3)];
            [-q(4) q(1) q(2)];
            [q(3) -q(2) q(1)]];
        
    
    O = zeros(3);
    I = eye(3);
    
%     left = eye(10,13);%[[I O O O zeros(3,4)];
%             %[O I O O zeros(3,4)];
%             %[zeros(7,3) zeros(7,3) eye(7) zeros(7,3)]];
%         
%     right =  [[I O O O O ];
%             [O I O O O];
%             [zeros(4,3) Q_dtheta zeros(4,6)];
%             [O O O I]];
     %[[I O O O O ]; %10x12
        %[zeros(4,3)  Q_dtheta zeros(4,9)]];
      C  =[[eye(3) zeros(3,12)];
            [zeros(4,3) Q_dtheta zeros(4,9)]];
        
    z = [pos; q];
    W = eye(7,7);
     %[ mu, Sigma, X ] = update2(C, mu_bar,Sigma_bar, X_bar, W, R_noise, z,R);
     
      %previous_mu = mu;
      %previous_Sigma = Sigma;
      %nom_state = X;
     prev_time = sensor.t;
end
 


end

