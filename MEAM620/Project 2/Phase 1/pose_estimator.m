init_script;
figure
vec = zeros(0,3);
qs= zeros(0,4);
n = 864;
for i = 1:n
[p,q] = estimate_pose(data(i), K, XYZ, Yaw, tags,splits);
vec = [vec; p'];

qs = [qs; q'];
end
plot3(vec(:,1), vec(:,2),vec(:,3));
hold on
plot3(vicon(1,:), vicon(2,:), vicon(3,:));
figure
plot(qs(2,:));
%error = norm(vec-vicon(1:3,1:n)');
