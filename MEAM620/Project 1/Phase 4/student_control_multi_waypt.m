% MEAM 620 Student Multi Waypoint code

if (setitM(qn)~=903) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=903;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    isCircle = false;
    flag = 1;
    load('test_waypoints/test_waypts_1.mat');
    path = [qd{qn}.pos'; waypts];

    trajectory_generator([], [], 0, path );
    
%     qd.pos_des = desired_state.pos;
%     qd.vel_des = desired_state.vel;
%     qd.acc_des = desired_state.acc;
    
    t0 = GetUnixTime;
    des_pos_save =  zeros(3,1);
    pos_save =  zeros(3,1);
    params = nanoplus();
    qd{qn}.yaw_des = 0;
%     qd{qn}.acc_des = zeros(3,1);
%     qd{qn}.vel_des = zeros(3,1);
    
    %END INTITIALIZATION

end %everything beyond this point runs every control loop iteration

t = GetUnixTime - t0;

%if t < 5
    [desired_state] = trajectory_generator(t, qn);    
% else
%     if isCircle == true
%         [desired_state] = circle(t-5, qn);
%     elseif flag == 1
%         flag = 0;
%         path = [qd{qn}.pos'; waypts];
%         [desired_state] = trajectory_generator(t, qn, [], path(2:end,:));
%     else
%         [desired_state] = trajectory_generator(t, qn);
%     end
% end

qd{qn}.pos_des = desired_state.pos;
qd{qn}.vel_des = desired_state.vel;
qd{qn}.acc_des = desired_state.acc;

%COMPUTE CONTROL HERE
[F, M, trpy, drpy] = controller(qd, t, qn, params);
des_pos_save(:,end+1) = qd{qn}.pos_des;
pos_save(:,end+1) = qd{qn}.pos;
if mod(t,1) == 0
    realtime_plot( t, ViconData, trpySave, des_pos_save );
end
