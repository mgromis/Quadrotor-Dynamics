function [X, Z] = eskf_map(sensor, K_camera,XYZ, yaw, tags,splits)
% ESKF Error State Kalman Filter with IMU as inputs
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




persistent prev_time dt previous_mu previous_Sigma nom_state R F U...
    mu_bar Sigma_bar state_ind X_bar Q size_state size_error_state state_map err_ind 
    


X = zeros(16,1);
Z = [];
alpha = .8;
init_cov = .1;

if(isempty(prev_time) && ~isempty(sensor)) %initialization
    size_state = 16;
    size_error_state = 15;
    pos = [0 0 0]';
    q = [1 0 0 0]';
    vel = [0 0 0]';
    state_ind = [];
    err_ind = [];
    R = quat_to_mat(q);
    dt = .01;
    prev_time = sensor.t;
    nom_state = [pos; vel; q; zeros(6,1)];
    X = nom_state;
    previous_mu = zeros(size_error_state,1);
    previous_Sigma = zeros(size_error_state);
    state_map = zeros(108,25);
    return
    
    
end
X = zeros(size_state,1);

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
    F_prime = [[I I*dt O O O];
        [O I -R*Acc_SS*dt -R*dt O];
        [O O R'*omg_SS O -R*dt];
        [O O O I O];
        [O O O O I]];
    F = [[F_prime zeros(size(F_prime,1),size_error_state-size(F_prime,1))];
        [zeros(size_error_state-size(F_prime,2),size(F_prime,2)) eye(size_error_state-size(F_prime,1))]];
    
    
    U = [[O O O O];
        [R O O O];
        [O R O O ];
        [O O R O];
        [O O O O];
        zeros(size_error_state-15,12)];
    
    
    [ mu_bar, Sigma_bar, X_bar ] = prediction(F, previous_mu, previous_Sigma, U, Q, nom_state, sensor, dt,R);
    previous_mu = mu_bar;
    previous_Sigma = Sigma_bar;
    nom_state = X_bar;
    prev_time = sensor.t;
    X = nom_state; %X_true_bar; %add on error state
    R = quat_to_mat(X(7:10));
end


% if(~isempty(prev_time) && ~isempty([sensor.id])) %perform update
%     
%     id_ind = sensor.id+1;
%     n = sum(state_map(id_ind,1));
%     tag_ind = find(state_map(id_ind,1) ==1);
%     lambda_cov = 10;
%     if n~=0
%         
%         %begin building C matrix and Z measured
%         z_p = zeros(n*8, 1);
%         C = zeros(n*8, size_error_state); 
%         rx = [1 0 0;
%             0 cos(pi) -sin(pi);
%             0 sin(pi) cos(pi)];
%         rz = [cos(-yaw) -sin(-yaw) 0;
%             sin(-yaw) cos(-yaw) 0;
%             0 0 1];
%         for tag = 1:n
%             for corner = 1:4
%                 
%                 %Build rotations and translations required for estimating
%                 %world pose
%                 current_tag_id = id_ind(tag_ind(tag));
%                 features = state_map(current_tag_id,(3*(3+(corner-1)):(11+3*(corner-1))))';
%                 xf = features(1);
%                 yf = features(2);
%                 lambda = features(3);
%                 wRk = quat_to_mat(state_map(current_tag_id,5:8));
%                 wPk =  (state_map(current_tag_id,2:4));
%                 bPc = XYZ';
%                 wRb = quat_to_mat(nom_state(7:10)');
%                 wPb = nom_state(1:3);
%                 bRc = rz*rx;
%                 
% 
%                 world = get_world_estimate(wRk, bRc, lambda, xf, yf, bPc, wPk');
%                 X = world(1);
%                 Y = world(2);
%                 Z = world(3);
%                 Jp = 1/Z*[[1 0 -X/Z];
%                     [0 1 -Y/Z]];
%                 %Create components of C matrix
%                 dz_dp = -Jp*bRc'*wRb';
%                 dz_dtheta = Jp*bRc'*wRb'*hat(world-wPb);
%                 dz_dp_k = dz_dp;
%                 dz_dtheta_k = -Jp*bRc'*wRb'*hat(world);
%                 dz_dxf = Jp*bRc'*wRb'*wRk*bRc*exp(lambda)*[[1 0 xf]; [0 1 yf]; [0 0 1]];
%                 row_start = 8*(tag-1)+2*corner-1;
%                 row_end = 8*(tag-1)+2*corner;
% 
%                 %Order error-pos related components of C
%                 C(row_start:row_end,1:15) = ...
%                     [dz_dp zeros(2,3) dz_dtheta zeros(2,6)];
%                 
%                 %Order keyframe_error related components of C
%                 frame_ind = state_map(id_ind(tag_ind(tag)),21)';
%                 C(row_start:row_end,frame_ind:(frame_ind+5)) =...
%                     [dz_dp_k, dz_dtheta_k];
%                 feature_location = state_map(id_ind(tag_ind(tag)),23);
%                 if state_map(id_ind(tag_ind(tag)),22)==1
%                     feature_location = 3*(corner-1)+feature_location;
%                     C(row_start:row_end,feature_location:feature_location+2)=dz_dxf;
%                     
%                 elseif corner == 4 
%                     feature_location = feature_location+3;
%                     C(row_start:row_end,feature_location:feature_location+2)=dz_dxf;
%                 else
%                     feature_location = feature_location+ corner-1;
%                     C(row_start:row_end,feature_location) = dz_dxf(:,3);
%                 end 
%                 %Build z_p
%                 z_p(row_start:row_end) = ...
%                     proj(bRc'*(wRb'*(wRk*(bRc*exp(lambda)*[xf;yf;1]+bPc)+wPk')-wRb'*wPb)-bRc'*bPc);
%             end
%         end
%         %Calibrate measured points and build z_m
%         cal_1 = K_camera\[sensor.p1(:,tag_ind); ones(1,n)];
%         cal_2 = K_camera\[sensor.p2(:,tag_ind); ones(1,n)];
%         cal_3 = K_camera\[sensor.p3(:,tag_ind); ones(1,n)];
%         cal_4 = K_camera\[sensor.p4(:,tag_ind); ones(1,n)];
%         cal_1 = cal_1(1:2,:);
%         cal_2 = cal_2(1:2,:);
%         cal_3 = cal_3(1:2,:);
%         cal_4 = cal_4(1:2,:);
%         
%         
%         
%         z_m = [cal_1; cal_2; cal_3; cal_4];
%         z_m = z_m(:);
%         
%         
%         sgm = .1;
%         
%         R_noise  = eye(1)*sgm^2;
%         W = eye(size(C));
%         [ mu, Sigma, X ] = update(C, mu_bar,Sigma_bar, X_bar, W, R_noise, z_m, z_p,...
%             size_error_state, state_ind, err_ind, state_map );
%         
%         
%         previous_mu = mu;
%         previous_Sigma = Sigma;
%         nom_state = X;
%         
%     end
%     %now update observed landmarks
%     prev_time = sensor.t;
%     
%     key_frame = [1, nom_state(1:3)', nom_state(7:10)'];
%     visited = state_map(:,1);
%     id_ind = sensor.id+1;
%     num_new_tags = sum(visited(id_ind)~=1);
%     tag_ind = find(visited(id_ind,1) ==0);
%     
%     if num_new_tags>0
%         points = zeros(num_new_tags,12);
%         points(:,1:3) = transpose(K_camera\[sensor.p1(:,tag_ind); ones(1,num_new_tags)]);
%         points(:,4:6) = transpose(K_camera\[sensor.p2(:,tag_ind); ones(1,num_new_tags)]);
%         points(:,7:9) = transpose(K_camera\[sensor.p3(:,tag_ind); ones(1,num_new_tags)]);
%         points(:,10:12) = transpose(K_camera\[sensor.p4(:,tag_ind); ones(1,num_new_tags)]);
%         points(:,[3 6 9 12]) = 0;
%         dkind = ones(num_new_tags,1)*size_error_state+1;
%         has_dxfdyf = [0; ones(num_new_tags-1,1)];
%         feat1_idx = (1:num_new_tags)'*12-4+size_state;
%         dfeat1_idx = [size_error_state+7; (1:num_new_tags-1)'*12+1+size_error_state];
%         keyframe_num = ones(num_new_tags,1)*(size(state_ind,1)+1);
%         state_map(id_ind(tag_ind),:) = [repmat(key_frame,num_new_tags,1),points,...
%             dkind, has_dxfdyf,dfeat1_idx,feat1_idx,keyframe_num];
%         
%         vec = [nom_state(1:3); nom_state(7:10);...
%             reshape(points',[12*num_new_tags,1])];
%         nom_state = [nom_state; vec];
%         state_ind = [state_ind; size_state+1];
%         size_state = size(nom_state,1);
%         
%         error_state_vec = [previous_mu(1:3); previous_mu(7:9);...
%             zeros(size(vec,1)-13,1)];
%         previous_mu = [previous_mu; error_state_vec];
%         err_ind = [err_ind; size_error_state+1];
%         size_error_state = size(previous_mu,1);
%         
%         n = size(previous_Sigma,1);
%         error_state_cov = ones(size_error_state)*init_cov;
%         error_state_cov(1:n,1:n) = previous_Sigma;
%         error_state_cov(1:n,n+1:n+6) = [previous_Sigma(:,1:3), previous_Sigma(:,7:9)];
%         error_state_cov(n+1:n+6, 1:n) = [previous_Sigma(1:3,:); previous_Sigma(7:9, :)];
%         error_state_cov(n+1:n+6,n+1:n+6) = [previous_Sigma(1:3,1:3), previous_Sigma(4:6, 7:9);...
%             previous_Sigma(7:9,4:9)];
%         
%         error_state_cov([n+7:n+9,n+12],:) = lambda_cov;
%         error_state_cov(:,[n+7:n+9,n+12]) = lambda_cov;
%         lambdas = (1:(num_new_tags-1)*4)*3+12+n;
%         error_state_cov(lambdas,:) = lambda_cov;
%         error_state_cov(:,lambdas) = lambda_cov;
%         
%         previous_Sigma = error_state_cov;
%         
%     end
%     X = nom_state;
%     R = quat_to_mat(X(7:10)');
%     
%     
%     
% end
end

