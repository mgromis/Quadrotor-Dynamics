init_script
clear eskf2
cam_times = [data.t];


close all
figure
all_states = [];
all_pos = [];
for i = 1:size(cam_times,2)
    sensor = data(i);
    [X, Z] = eskf2(sensor, K_camera,XYZ, yaw, tags,splits);
    vec = [X; cam_times(i)];
    all_states = [all_states vec];
    
end

%% Plot XYZ
figure
subplot(3,1,1)
plot(cam_times, all_states(1,:))
hold on
plot(vic_times, vicon(1,:))
title('X')

subplot(3,1,2)
plot(cam_times, all_states(2,:))
hold on
plot(vic_times, vicon(2,:))
title('Y')

subplot(3,1,3)
plot(cam_times, all_states(3,:))
hold on
plot(vic_times, vicon(3,:))
legend('measured', 'vicon');
title('Z')

%% Plot RYP (ish)

%eul = quat2eul(all_states(7:10,:)');

figure
subplot(3,1,1)
plot(cam_times, all_states(8,:))
hold on
plot(vic_times, vicon(4,:))
title('roll')

subplot(3,1,2)
plot(cam_times, all_states(9,:))
hold on
plot(vic_times, vicon(5,:))
title('pitch')

subplot(3,1,3)
plot(cam_times, all_states(10,:))
hold on
plot(vic_times, vicon(6,:))
legend('measured', 'vicon');
title('yaw')

%% Plot XYZ dot
figure
subplot(3,1,1)
plot(cam_times, all_states(4,:))
hold on
plot(vic_times, vicon(7,:))
title('Xdot')

subplot(3,1,2)
plot(cam_times, all_states(5,:))
hold on
plot(vic_times, vicon(8,:))
title('Ydot')

subplot(3,1,3)
plot(cam_times, all_states(6,:))
hold on
plot(vic_times, vicon(9,:))
legend('measured', 'vicon');
title('Zdot')