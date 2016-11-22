function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
figure
axis vis3d equal
axis([map.data(1,1) map.data(1,4) map.data(1,2) map.data(1,5) map.data(1,3) map.data(1,6)  ])
hold on
map_size = size(map.obst);


for i = 1:map_size(1) 
    
p1 = map.obst(i,1:3);
p2 = [map.obst(i,4) map.obst(i,2) map.obst(i,3)];
p4 = [map.obst(i,1) map.obst(i,5) map.obst(i,3)];
p3 = [map.obst(i,4) map.obst(i,5) map.obst(i,3)];
p5 = [map.obst(i,1) map.obst(i,2) map.obst(i,6)];
p6 = [map.obst(i,4) map.obst(i,2) map.obst(i,6)];
p8 = [map.obst(i,1) map.obst(i,5) map.obst(i,6)];
p7 = map.obst(i,4:6);

color = map.obst(i,7:9);
color = color/255.0;

    
x = [p1(1) p2(1) p3(1) p4(1)];
y = [p1(2) p2(2) p3(2) p4(2)];
z = [p1(3) p2(3) p3(3) p4(3)];

fill3(x, y, z, color);
xlabel('x'); ylabel('y'); zlabel('z');  
   
grid
hold on
x = [p5(1) p6(1) p7(1) p8(1)];
y = [p5(2) p6(2) p7(2) p8(2)];
z = [p5(3) p6(3) p7(3) p8(3)];
h = fill3(x, y, z, color);
x = [p2(1) p6(1) p7(1) p3(1)];
y = [p2(2) p6(2) p7(2) p3(2)];
z = [p2(3) p6(3) p7(3) p3(3)];
h =fill3(x, y, z, color);
 

x = [p2(1) p6(1) p5(1) p1(1)];
y = [p2(2) p6(2) p5(2) p1(2)];
z = [p2(3) p6(3) p5(3) p1(3)];
h =fill3(x, y, z, color);

x = [p1(1) p5(1) p8(1) p4(1)];
y = [p1(2) p5(2) p8(2) p4(2)];
z = [p1(3) p5(3) p8(3) p4(3)];
h = fill3(x, y, z, color);

x = [p4(1) p8(1) p7(1) p3(1)];
y = [p4(2) p8(2) p7(2) p3(2)];
z = [p4(3) p8(3) p7(3) p3(3)];
h = fill3(x, y, z, color);
end
plot3(path(:,1), path(:,2),path(:,3))


end




