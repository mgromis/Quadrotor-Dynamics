map = load_map('map2.txt', 0.1, .1, 0);
start1 = [0.0  0 0];
stop1  = [0  1 1];
start2 = stop1;
stop2 = [1 1 0];
start3 = stop2;
stop3 = start1;
[a1,b] =dijkstra(map,start1,stop1,true);
[a2,b] =dijkstra(map,start2,stop2,true);
[a3,b] =dijkstra(map,start3,stop3,true);
%plot_path(map,i)
%average_vel = 2;
%dt = abs(diff(i)/average_vel);
figure
hold on
T = 30;
time = linspace(0,T,size(a1,1))';
n = 20;
fit_x = polyfit(time,a1(:,1),n);
fit_y = polyfit(time,a1(:,2),n);
fit_z = polyfit(time,a1(:,3),n);
x =polyval(fit_x,time);
y =polyval(fit_y,time);
z =polyval(fit_z,time);


% vel_x = polyder(fit_x);
% vel_y = polyder(fit_y);
% vel_z = polyder(fit_z);
% acc_x = polyder(vel_x);
% acc_y = polyder(vel_y);
% acc_z = polyder(vel_z);





course = [x, y, z];
plot3(x,y,z);


time = linspace(0,T,size(a1,1))';
n = 20;
fit_x = polyfit(time,a2(:,1),n);
fit_y = polyfit(time,a2(:,2),n);
fit_z = polyfit(time,a2(:,3),n);
x =polyval(fit_x,time);
y =polyval(fit_y,time);
z =polyval(fit_z,time);
hold on
course = [x, y, z];
plot3(x, y, z);



time = linspace(0,T,size(a1,1))';
n = 20;
fit_x = polyfit(time,a3(:,1),n);
fit_y = polyfit(time,a3(:,2),n);
fit_z = polyfit(time,a3(:,3),n);
x =polyval(fit_x,time);
y =polyval(fit_y,time);
z =polyval(fit_z,time);

course = [x, y, z];
plot3(x, y, z);







