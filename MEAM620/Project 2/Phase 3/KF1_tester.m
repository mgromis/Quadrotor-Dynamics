init_script
clear eskf1
vic_times = [time];
cam_times = [data.t];
sorted_times = union(vic_times, cam_times);

close all
figure
all_states = [];
all_pos = [];
for i = 1:size(sorted_times,2)
    ind_vic = find(sorted_times(i)==vic_times);
    ind_cam = find(sorted_times(i)==cam_times);
    vic.t = vic_times(ind_vic);
    vic.vel = vicon(7:end,ind_vic);
    sensor = data(ind_cam);
    [X, Z] = eskf1(sensor, vic, K_camera,XYZ, yaw, tags,splits);
    if(~isempty(sensor))
    [pos, q] = estimate_pose(sensor, K_camera, XYZ, yaw, tags,splits);
    all_pos = [all_pos pos];
    end
    vec = [X; sorted_times(i)];
    all_states = [all_states vec];
    
end


subplot(3,1,1)
ylabel('X')
plot(sorted_times, all_states(1,:))
hold on
plot(vic_times, vicon(1,:))

subplot(3,1,2)
ylabel('Y')
plot(sorted_times, all_states(2,:))
hold on
plot(vic_times, vicon(2,:))

subplot(3,1,3)
zlabel('Z')
plot(sorted_times, all_states(3,:))
hold on
plot(vic_times, vicon(3,:))
legend('measured', 'vicon');

figure
subplot(3,1,1)
title('roll')
plot(sorted_times, all_states(5,:))
hold on
plot(vic_times, vicon(4,:))

subplot(3,1,2)
title('pitch')
plot(sorted_times, all_states(6,:))
hold on
plot(vic_times, vicon(5,:))

subplot(3,1,3)
title('yaw')
plot(vic_times, all_states(7,:))
hold on
plot(time, vicon(6,:))
legend('measured', 'vicon');


% plot3(all_states(1,:), all_states(2,:), all_states(3,:))
% %plot3(all_pos(1,:), all_pos(2,:), all_pos(3,:))
% %plot(vic_times, vicon(2,:));
% %hold on 
% %plot(cam_times, all_pos(2,:));
% %hold on
% %plot(sorted_times, all_states(2,:));
% 
% hold on
% plot3(vicon(1,:), vicon(2,:), vicon(3,:));