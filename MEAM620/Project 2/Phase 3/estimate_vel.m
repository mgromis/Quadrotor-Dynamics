function [vel, omg] = estimate_vel(sensor, K, XYZ, yaw, tags,splits)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
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
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor


persistent dt prev_corners prev_img prev_t 

vel = zeros(3,1);
omg = zeros(3,1);
n =50; %number of points to sample
c = .8;

if(isempty(prev_t)) %no previous time, set to average time step for dt
    dt = .0205;
else
    dt = (sensor.t-prev_t)*(1-c)+dt*(c);
end

if(isempty(sensor.id)) %if there are no corners, need to set previous variables to null (except for time)
    prev_img = [];
    prev_t = sensor.t;
    prev_corners = [];
    vel = [];
    omg = [];
    return;
end

if(~isempty(prev_img)) %if we have a previous image, we perform optical flow
    
    [pos, R] = estimate_pose_2(sensor, K, XYZ, yaw, tags,splits);
    track = vision.PointTracker;
    initialize(track, prev_corners, prev_img);
    [tracked, val] = step(track, sensor.img);
    
    num_points = size(tracked,1);
    tracked = [tracked'; ones(1,num_points)];
    prev_tracked = [prev_corners'; ones(1,num_points)];
    
    current_points = K\tracked;
    current_points = (current_points(1:3,val))';
    prev_points = K\prev_tracked;
    prev_points = (prev_points(1:3,val))';
    
    A = zeros(2*size(current_points,1),6);
    R2 = R';
    H = [R2(:,1) R2(:,2) pos]; 
    flow = (current_points-prev_points)/dt;
    sorted_flow = zeros(2*size(current_points,1),1);
    counter = 1;
    for i = 1:size(current_points,1)
        
        Z = H\((current_points(i,:))');
        z = -1/Z(3);
        x = current_points(i, 1);
        y = current_points(i,2);
        A((counter):(counter+1),:) = [[-1/z 0 x/z x*y -(1+x^2) y];
            [0 -1/z  y/z 1+y^2 -x*y -x]];
        sorted_flow((counter):(counter+1),:)= [flow(i,1); flow(i,2)];
        counter = counter +2;
    end
    
    [new_flow, new_A] = ransac(3, flow, .002, A, sorted_flow, false);
    v = new_A\new_flow;
    prev_img = sensor.img;
    corners = detectFASTFeatures(prev_img);
    prev_corners = corners.selectStrongest(n).Location;
    prev_t = sensor.t;
    vel = R*v(1:3);
    omg = R*v(4:end);
    return;
end
%execute when there is no previous image
prev_t = sensor.t;
prev_img = sensor.img;
corners = detectFASTFeatures(prev_img);
prev_corners = corners.selectStrongest(n).Location;
end

