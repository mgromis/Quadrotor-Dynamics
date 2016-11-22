close all
init_script;
figure
vec = zeros(0,3);
qs= zeros(0,3);
n = size([data.t],2);
a = .1;
%[v1, o1] = estimate_vel(data(2), K, XYZ, yaw, tags,splits);
%filtered_v = v1';
%filtered_v_vec = [];
for i = 1:n
[vel, omg] = estimate_vel(data(i), K, XYZ, yaw, tags,splits);
vec = [vec; vel'];
%filtered_v = (vel')*a + filtered_v*(1-a);
%filtered_v_vec = [filtered_v_vec; filtered_v];
qs = [qs; omg'];
end
sens_times = [data.t]';

plot(sens_times, vec(:,1))
hold on

plot(time, vicon(7,:))
xlabel('t')
ylabel('xdot')
figure
plot(sens_times, vec(:,2))
hold on
plot(time, vicon(8,:))
xlabel('t')
ylabel('ydot')
figure
plot(sens_times, vec(:,3))
hold on

plot(time, vicon(9,:))
xlabel('t')
ylabel('zdot')
figure
plot(sens_times, qs(:,1))
hold on
xlabel('t')
ylabel('omega_x')
plot(time, vicon(10,:))

figure
plot(sens_times, qs(:,2))
hold on

plot(time, vicon(11,:))
xlabel('t')
ylabel('omega_y')

figure
plot(sens_times, qs(:,3))
hold on
plot(time, vicon(12,:))
xlabel('t')
ylabel('omega_z')


%error = norm(vec-vicon(1:3,1:n)');
