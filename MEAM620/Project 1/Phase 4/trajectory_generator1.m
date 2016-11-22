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

persistent map0 path0 time_store

if nargin > 2
    map0 = map;
    path0 = path;
    
    %path0 = safe_short_path(map,path0);
    
    %plot_path(map0,path0);
    
    max_acc = 2.0; % 4.3164; % 24.525;
    N = length(path0);
    
    time_store = ones(N,1);
    for i = 1:N,
        time_store(i) = time_store(i)*20*(i-1);
    end
    
    for j = 2:N,
        p0 = path0(j-1,:);
        pf = path0(j,:);
        dist = norm(pf - p0);
        
        if dist < 1.5
            avg_vel = dist;
        else
            avg_vel = dist * 0.4;
        end
        
        time_store(j) = time_store(j-1) + dist / avg_vel;
        t0 = time_store(j-1);
        tf = time_store(j);
        
        for i = 1:20,
            
            t = dist / avg_vel;
            
            quintic = [
                1    t0  t0^2     t0^3  t0^4        t0^5;
                0     1  2*t0   3*t0^2  4*t0^3    5*t0^4;
                0     0     2     6*t0  12*t0^2  20*t0^3;
                1    tf  tf^2     tf^3  tf^4        tf^5; 
                0     1  2*tf   3*tf^2  4*tf^3    5*tf^4;
                0     0     2     6*tf  12*tf^2  20*tf^3;
                ];
            
            conditions = [p0; 0 0 0; 0 0 0; pf; 0 0 0; 0 0 0];
            coeffs = quintic \ conditions;
            % Pull out individual coefficients.
            %a0 = coeffs(1,:);
            a1 = coeffs(2,:);
            a2 = coeffs(3,:);
            a3 = coeffs(4,:);
            a4 = coeffs(5,:);
            a5 = coeffs(6,:);
            
            acc_n = 2*max_acc;
            for tt = linspace(0,t,ceil(dist*10)),
                %pos = (a0 + a1*tt + a2*tt^2 + a3*tt^3 + a4*tt^4 + a5*tt^5)';
                vel = (a1 + 2*a2*tt + 3*a3*tt^2 + 4*a4*tt^3 + 5*a5*tt^4)';
                acc = (2*a2' + 6*a3'*tt + 12*a4'*tt^2 + 20*a5'*tt^3)';
                acc_n = sqrt(sum(acc.^2,2));
                if any(acc_n > max_acc)
                    avg_vel = avg_vel * 0.97;
                    break;
                end
            end
            
            if acc_n <= max_acc
                time_store(j) = t;
                break;
            end
        end
        
        
    end
    
    return;
end

% if t > 4.57
%    disp(t); 
% end

pos = zeros(3,1);
vel = zeros(3,1);
acc = zeros(3,1);

j = find(time_store > t, 1, 'first');

if isempty(j)
    j = length(time_store);
end

p0 = path0(j-1,:);
pf = path0(j,:);
t0 = time_store(j-1);
tf = time_store(j);

if t >= t0 && t < tf

    quintic = [1    t0  t0^2     t0^3  t0^4        t0^5;
               0     1  2*t0   3*t0^2  4*t0^3    5*t0^4;
               0     0     2     6*t0  12*t0^2  20*t0^3;
               1    tf  tf^2     tf^3  tf^4        tf^5; 
               0     1  2*tf   3*tf^2  4*tf^3    5*tf^4;
               0     0     2     6*tf  12*tf^2  20*tf^3;
               ];

    conditions = [p0; 0 0 0; 0 0 0; pf; 0 0 0; 0 0 0];
    coeffs = quintic \ conditions;
    % Pull out individual coefficients.
    a0 = coeffs(1,:);
    a1 = coeffs(2,:);
    a2 = coeffs(3,:);
    a3 = coeffs(4,:);
    a4 = coeffs(5,:);
    a5 = coeffs(6,:);

    pos = (a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5)';
    vel = (a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4)';
    acc = (2*a2' + 6*a3'*t + 12*a4'*t^2 + 20*a5'*t^3)';
end

if t >= time_store(end)
    pos = path0(end,:);
    %vel = [0 0 0];
    %acc = [0 0 0];
end

yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
