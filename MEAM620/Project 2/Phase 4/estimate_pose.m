function [pos, q] = estimate_pose(sensor, K, XYZ, yaw, tags,splits, state_map)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings, you should not use these in this phase
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
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k

pos = zeros(3,1);
q = eye(3);
visited = state_map(:,1);
id_ind = sensor.id+1;
n = sum(visited(id_ind));
px_ind = find(visited(id_ind,1) ==1);

%n = size(find(state_map(:,1)==1),2);


if(n>0)
    
rx = [1 0 0;
       0 cos(pi) -sin(pi);
       0 sin(pi) cos(pi)];
   rz = [cos(-yaw) -sin(-yaw) 0;
       sin(-yaw) cos(-yaw) 0;
       0 0 1];    
    
    
    
%Corners = zeros(2,n,4);
Corners(:,:,1) = sensor.p1(:,px_ind);
Corners(:,:,2) = sensor.p2(:,px_ind);
Corners(:,:,3) = sensor.p3(:,px_ind);
Corners(:,:,4) = sensor.p4(:,px_ind);
%Corners(:,:,5) = sensor.p0;
D = zeros(2*n*4,9);
counter =1;
    for i = 1:n
        for j = 1:4
            current_tag_id = id_ind(px_ind(i));
            
            features = state_map(current_tag_id,(9+3*(j-1):(11+3*(j-1))))';
            xf = features(1);
            yf = features(2);
            lambda = features(3);
            
            wRk = quat_to_mat(state_map(current_tag_id,5:8));
            wPk =  (state_map(current_tag_id,2:4));
            bPc = XYZ';
            u = Corners(1,i,j);
            v = Corners(2,i,j);

            calib = K\[u; v; 1];
            bRc = rz*rx;
            world = get_world_estimate(wRk, bRc, lambda, xf, yf, bPc, wPk'); 
            X = world(1);
            Y = world(2);
            %[X,Y] = getCoords(sensor.id(i), j,splits, tags);
            D(counter:(counter+1),:) = [[-X -Y -1 0 0 0 X*calib(1) calib(1)*Y calib(1)];
                [0 0 0 -X -Y -1 calib(2)*X calib(2)*Y calib(2)]];
            counter = counter +2;
            
         
        end
    end
    
    
    [~, ~, V] = svd(D);
    nor = [V(1,9) V(4,9) V(7,9)];
    A = [V(1:3,9)'; V(4:6,9)'; V(7:9,9)']/norm(nor);
    A = (1-2*(A(end,end)<0))*A;
    
    
   [U, ~, V] = svd([A(:,1:2) cross(A(:,1),A(:,2))]);
   
   S = [1 0 0;
       0 1 0;
       0 0 det(U*V')];
   
  R = U*S*V';
   R = R';
   T = A(:,3);
   T = R*(T);
   pos = T;
   q = R;

end
end











   