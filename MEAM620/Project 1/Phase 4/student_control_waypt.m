% MEAM 620 Student One Waypoint code

if (setitM(qn)~=902) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=902;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    
    params = nanoplus();
    
    %[ desired_state ] = trajectory_generator([], [], map, path);
    %[desired_state] = diamond(t, qn);
    initial_pos = [0, 0, 0]';
    final_pos = [1, 1, .5]';
    T = 20;
    [desired_state] = single_point(T, qn, qd, initial_pos, final_pos);
    qd{qn}.pos_des = desired_state.pos;
    qd{qn}.vel_des = desired_state.vel;
    qd{qn}.acc_des = desired_state.acc;
    qd{qn}.yaw_des = desired_state.yaw;
    
    t0 = GetUnixTime;
    des_pos_save = zeros(3,1);
    pos_save =  zeros(3,1);
    %END INTITIALIZATION

end %everything beyond this point runs every control loop iteration

t = GetUnixTime - t0;

[desired_state] = single_point(t, qn, qd);
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