function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;
persistent map0 path0 trajs T time  



if(nargin==4)
    
    map0 = map;
    path0 = simplify(path{1}, map);
    
    
    distances = zeros(size(path0,1)-1,1);
    log_d = zeros(size(path0,1)-1,1);
    scaled = zeros(size(distances));
    for i=1:size(distances,1)
        distances(i,1) = norm(path0(i,:)- path0(i+1,:));
        log_d(i,1) = log(norm(path0(i,:)- path0(i+1,:)));
        
    end
     for i=1:size(distances,1)
    scaled(i) = distances(i)^.5;
    
     end
     
    time = zeros(size(distances));
    total_distance = sum(distances);
    average_vel = 1.65;
    T = total_distance/average_vel;
    dt = size(distances);
    for i = 1:size(distances,1)
        time(i) = sum(distances(1:i))*T/(total_distance);
        dt(i) = scaled(i)*T/(sum(scaled));
    end
    for i = 1:size(distances,1)
        time(i) = sum(dt(1:i));
    end
        
    time = [0;time]
    psize =size(path0);
    trajs = zeros(3,6,psize(1)-3);
    vels = zeros(psize(1)-1,3);
    vel_factor = 1;
    for i =1:(size(path0,1)-1)
        vels(i,:) = (path0(i+1,:)-path0(i,:))/(norm(path0(i+1,:)-path0(i,:)))*vel_factor;
    end
    
    vels = diff(path0(:,:))./([diff(time),diff(time),diff(time)]);
    
    
    for k = 2:(psize(1)-2)
        trajs(:,:,k) = getQuintic(time(k),time(k+1),path0(k,:),path0(k+1,:), zeros(1,3),...
            zeros(1,3), zeros(1,3),zeros(1,3));
    end
    
    trajs(:,:,1) = getQuintic(time(1),time(2),path0(1,:),path0(2,:), zeros(1,3),...
        zeros(1,3), zeros(1,3),zeros(1,3));
    trajs = cat(3,trajs,getQuintic(time(size(path0,1)-1),time(size(path0,1)),...
        path0(psize(1)-1,:),path0(psize(1),:), zeros(1,3),zeros(1,3),...
        zeros(1,3),zeros(1,3)));
    
    
    
    %     for k = 2:(psize(1)-2)
    %          trajs(:,:,k) = getQuintic(time(k),time(k+1),path0(k,:),path0(k+1,:), vels(k-1,:),...
    %             vels(k,:), zeros(1,3),zeros(1,3));
    %     end
    %
    %         trajs(:,:,1) = getQuintic(time(1),time(2),path0(1,:),path0(2,:), zeros(1,3),...
    %             vels(1,:), zeros(1,3),zeros(1,3));
    %     trajs = cat(3,trajs,getQuintic(time(size(path0,1)-1),time(size(path0,1)),...
    %         path0(psize(1)-1,:),path0(psize(1),:), vels(end,:),zeros(1,3),...
    %         zeros(1,3),zeros(1,3)));
    %
    
    
    pos = [path0(1,1),path0(1,2),path0(1,3)];
    vel = [0,0,0];
    acc = [0, 0, 0];
    yaw = 0;
    yawdot = 0;
    
    
    
end

if(nargin==2)
    
    if(t>T)
        pos = path0(end,:);
        vel = [0,0,0];
        acc = [0, 0, 0];
        yaw = 0;
        yawdot = 0;
    else
        
        time_ind = find(time>t);
        traj = trajs(:, :, time_ind(1)-1);
        
        pos = [traj(1,1)+traj(1,2)*t+traj(1,3)*t^2+traj(1,4)*t^3+traj(1,5)*t^4+traj(1,6)*t^5;...
            traj(2,1)+traj(2,2)*t+traj(2,3)*t^2+traj(2,4)*t^3+traj(2,5)*t^4+traj(2,6)*t^5;...
            traj(3,1)+traj(3,2)*t+traj(3,3)*t^2+traj(3,4)*t^3+traj(3,5)*t^4+traj(3,6)*t^5];
        vel = [traj(1,2)+2*traj(1,3)*t+3*traj(1,4)*t^2+4*traj(1,5)*t^3+5*traj(1,6)*t^4;
            traj(2,2)+2*traj(2,3)*t+3*traj(2,4)*t^2+4*traj(2,5)*t^3+5*traj(2,6)*t^4;
            traj(3,2)+2*traj(3,3)*t+3*traj(3,4)*t^2+4*traj(3,5)*t^3+5*traj(3,6)*t^4];
        acc = [2*traj(1,3)+6*traj(1,4)*t+12*traj(1,5)*t^2+20*traj(1,6)*t^3;
            2*traj(2,3)+6*traj(2,4)*t+12*traj(2,5)*t^2+20*traj(2,6)*t^3;
            2*traj(3,3)+6*traj(3,4)*t+12*traj(3,5)*t^2+20*traj(3,6)*t^3];
        yaw = 0;
        yawdot = 0;
        
    end
    
end
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;


end







function [coeffs] = getQuintic(t1,t2,p1,p2,v1,v2,a1,a2)

coeffs = zeros(3, 6);
A = [1 t1 t1^2 t1^3 t1^4 t1^5;
    0 1 2*t1 3*t1^2 4*t1^3 5*t1^4;
    0 0 2 6*t1 12*t1^2 20*t1^3;
    1 t2 t2^2 t2^3 t2^4 t2^5;
    0 1 2*t2 3*t2^2 4*t2^3 5*t2^4;
    0 0 2 6*t2 12*t2^2 20*t2^3];


for k = 1:3
    c = [p1(k), v1(k), a1(k), p2(k), v2(k), a2(k)]';
    coeffs(k,:) = A\c;
    
end
end


function [newpath] = simplify(path, map)
s = size(path);
pathrev = flipud(path);
keep = pathrev(1,:);
I =1;
if (s(1)==2)
  newpath = path
  return;
    
end

while(I<s(1)-1)
  
    for K = (I+2):(s(1))
        
        xs = linspace(pathrev(I,1), pathrev(K,1),200);
        ys = linspace(pathrev(I,2), pathrev(K,2),200);
        zs = linspace(pathrev(I,3), pathrev(K,3),200);
        
        if(any(collide(map,[xs', ys', zs'])))
            keep = [keep; pathrev(K-1,:)];
            I= K-1;
            break
        end
        
        if(K==s(1))
            I = s(1);
            break
        end
        
        
    end
    
    
end

keep = [keep; pathrev(end,:)];
newpath = flipud(keep)

end

function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, i) touches an obstacle and is 0 otherwise.
xmin = map.data(1);
ymin =map.data(2);
zmin = map.data(3);
xmax = map.data(4);
ymax = map.data(5);
zmax = map.data(6);
margin = map.margin;

psize = size(points);
C = zeros(psize(1),1);
min = [xmin ymin zmin];
max = [xmax ymax zmax];


obst_size = size(map.obst);
for i =1:psize(1)
    hit_boarder = points(i,1)<xmin||points(i,1)>xmax||points(i,2)<ymin||points(i,2)>ymax...
        ||points(i,3)<zmin||points(i,3)>zmax;
    if(hit_boarder)
        C(i,1) = 1;
    else
        
        for k=1:obst_size(1)
            
            
            hitobst = points(i,1)>=(map.obst(k,1)-margin)&&...
                points(i,1)<=(map.obst(k,4)+margin)&&points(i,2)>=(map.obst(k,2)-margin)&&...
                points(i,2)<=(map.obst(k,5)+margin)&&points(i,3)>=(map.obst(k,3)-margin/2.0)&&...
                points(i,3)<=(map.obst(k,6)+margin/2.0);
            if(hitobst)
                
                C(i,1) = 1;
                break
            end
            
        end
    end
end
end




% function [newpath] = removePoints(path)
% path_temp = path;
% s = size(path);
% removal = [];
% for i=1:(s(1)-1)
%
%     unit = (path(i+1,:)-path(i,:))/norm(path(i+1,:)-path(i,:));
%
%     for k=(i+1):(s(1)-2)
%         unit_test = (path(k+1,:)-path(k,:))/norm(path(k+1,:)-path(k,:));
%         if(isequal(round(unit,4),round(unit_test,4)))
%             removal = [removal;k];
%
%         else
%             break
%
%         end
%     end
%
% end
% path_temp(removal,:) = [];
% newpath = path_temp;
% end











