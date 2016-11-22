% MEAM 620 Student Hover code

if (setitM(qn)~=901) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=901;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    %if ~isempty(seqM(qn).seq(seq_cntM(qn)).kp) %check
%         qd{qn}.kp = seqM(qn).seq(seq_cntM(qn)).kp; 
%     else
%         qd{qn}.kp = [10, 10, 15]; %use the default kp 
%     end
%     if ~isempty(seqM(qn).seq(seq_cntM(qn)).kd) %check 
%         qd{qn}.kp = seqM(qn).seq(seq_cntM(qn)).kd;
%     else
%         qd{qn}.kd = [5, 5, 7]; %use the default kp
%         %these are just numbers that I made up, you will have to tune your own controller
%     end
    if ~isempty(seqM(qn).seq(seq_cntM(qn)).zoffset) %check for a desired z offset 
        qd{qn}.pos_des = qd{qn}.pos + [0; 0; seqM(qn).seq(seq_cntM(qn)).zoffset]; 
        % add z offset to current position to be your new desired hover position
    end
    
    qd{qn}.yaw_des = 0;
    qd{qn}.acc_des = zeros(3,1);
    qd{qn}.vel_des = zeros(3,1);
    % Quadrotor model
    params = nanoplus();
    
    t0 = GetUnixTime;
    des_pos_save = zeros(3,1);
    pos_save = zeros(3,1);
    %END INTITIALIZATION

end %everything beyond this point runs every control loop iteration

t = GetUnixTime - t0;

%COMPUTE CONTROL HERE
[F, M, trpy, drpy] = controller(qd, t, qn, params);
des_pos_save(:,end+1) = qd{qn}.pos_des;
pos_save(:,end+1) = qd{qn}.pos;
if mod(t,1) == 0
    realtime_plot( t, ViconData, trpySave, des_pos_save );
end

