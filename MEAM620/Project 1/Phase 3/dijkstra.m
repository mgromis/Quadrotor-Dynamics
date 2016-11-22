function [path, num_expanded] = dijkstra(map, start, goal, astar)
if(nargin<4)
    astar=false;
end

num_expanded = 0;
path = zeros(0,3);
if(collide(map,start)||collide(map,goal)) %if start or goal is illegal, return
    'Start or Goal is Invalid';
    return
end
if(isequal(start,goal))
    path = start;
    return
end


xmin = map.data(1);
ymin = map.data(2);
zmin = map.data(3);
xmax = map.data(4);
ymax = map.data(5);
zmax = map.data(6);
xy_res = map.data(7);
z_res = map.data(8);




grid = meshgrid(xmin:xy_res:xmax, ymin:xy_res:ymax, zmin:z_res:zmax);
grid_size = size(grid);


oldstart =start;
oldgoal = goal;


if(z_res>(zmax-zmin))
    grid_size = [grid_size(1),grid_size(2),1];
end


[x, y, z] = meshgrid(1:grid_size(3),1:grid_size(2), 1:grid_size(1));
%
indices = sub2ind([grid_size(1),grid_size(2),grid_size(3)],z,y,x);

distance = Inf*ones(1,grid_size(1)*grid_size(2)*grid_size(3));
parent = zeros(1,grid_size(1)*grid_size(2)*grid_size(3));
visited = ones(1,grid_size(1)*grid_size(2)*grid_size(3));
heuristic = zeros(1,grid_size(1)*grid_size(2)*grid_size(3));

[xx, yy, zz] = meshgrid(xmin:xy_res:xmax, ymin:xy_res:ymax, zmin:z_res:zmax);

remainders = mod(round(start-([xmin, ymin, zmin]),10),[xy_res, xy_res, z_res]);
if(~isequal(remainders,[0 0 0])) %if start is out of resolution
    'start out of resolution';
    
    
    start = start-remainders;
end
if(collide(map,start))
    start = start+[xy_res,xy_res,z_res];
    
end



remainders = mod(round(goal-[xmin, ymin, zmin],10),[xy_res, xy_res, z_res]);
if(~isequal(remainders,[0 0 0])) %if goal is out of resolution
    'goal out of resolution';
    goal = goal-remainders;
    
end


row = int16((start(1,2)-ymin)/xy_res+1);
col =int16((start(1,1)-xmin)/xy_res+1);
depth = int16(((start(1,3)-zmin)/z_res)+1);
index = indices(col,depth,row);

start_index = index;
row = int16((goal(1,2)-ymin)/xy_res+1);
col =int16((goal(1,1)-xmin)/xy_res+1);
depth = int16(((goal(1,3)-zmin)/z_res+1));
index = indices(col,depth,row);





goal_index = index;
begin = 1;
while (max(visited)) ~=0 %todo, test case wehre all unvistied nodes are INF - no path
    num_expanded = 1+num_expanded;
    if begin
        u = start_index;
        u_dist = 0;
        begin = 0;
        num_expanded =1;
        if (astar)
            heuristic(u) = norm(start-goal);
        end
        
    else
        
        
        
        finalmask = visited.*(distance); %implement logical indexing - a = [1, 0, 1] b = (3,4,5) take b(a) to get (3,5)
        finalmask(finalmask==0) = NaN;
        finalmask = heuristic+finalmask;
        [u_dist,us] = min(finalmask);
        u = us(1,1);
        num_expanded=  num_expanded+1;
        
    end
    
    if u==goal_index
        break
    end
    visited(u) = 0;
    children = Children(xx,yy,zz, grid, map, [xx(u),yy(u),zz(u)], xy_res, z_res, visited,indices);
    
    for i=1:length(children)
        if (children(i)~=-1)
            point = [xx(u) yy(u) zz(u)];
            child = [xx((children(i))),yy((children(i))),...
                zz((children(i)))];
            d = norm(child-point);
            if ((u_dist + d) < distance(children(i)))
                distance(children(i)) = u_dist + d;
                parent(children(i)) = u;
                if(astar)
                    heuristic(children(i)) = norm(child-goal);
                end
                
            end
        end
        
        
    end
    
    
end
if(parent(goal_index)==0)%no path found
    'no path found';
    path = zeros(0,3);
    return
end

distance =0;
u = goal_index;
while parent(u)~=0
    
    point = [xx(u), yy(u), zz(u)];
    distance = distance +norm(point-[xx(parent(u)),...
        yy(parent(u)), zz(parent(u))]);
    path = [point; path];
    u = parent(u);
    if (isequal(path(1,:),start))
        break
    end
end
path = [start;path];
if(~isequal(oldstart,start))
    path = [oldstart;path];
end
if(~isequal(oldgoal,goal))
    path = [path;oldgoal];
end
end





function [ neighb_ind ] = Children(xx,yy,zz, grid, map, current, xy_res, z_res,visited,indices)
%CHILDREN Summary of this function goes here
%   Detailed explanation goes here
xmin = map.data(1);
ymin =map.data(2);
zmin = map.data(3);
xmax = map.data(4);
ymax = map.data(5);
zmax = map.data(6);

%
neighbors = [[current(1)-xy_res, current(2), current(3)]; %%-x
    [current(1)+xy_res, current(2), current(3)];... %+x
    [current(1), current(2)-xy_res, current(3)];...%-y
    [current(1), current(2)+xy_res, current(3)];...%+y
    [current(1), current(2), current(3)-z_res];...%-z
    [current(1), current(2), current(3)+z_res]];...%+z  %
    % neighbors =[[current(1)-xy_res, current(2)-xy_res, current(3)+z_res];... % -x -y z+
%     [current(1)-xy_res, current(2), current(3)+z_res];... % -x y z+
%     [current(1)-xy_res, current(2)+xy_res, current(3)+z_res];... % -x +y z+
%     [current(1), current(2)+xy_res, current(3)+z_res];... % x y+ z+
%     [current(1), current(2), current(3)+z_res];... % x y z+
%     [current(1), current(2)-xy_res, current(3)+z_res];...x y- z+
%     [current(1)+xy_res, current(2)+xy_res, current(3)+z_res];...x+ y+ z+
%     [current(1)+xy_res, current(2), current(3)+z_res];...x+ y z+
%     [current(1)+xy_res, current(2)-xy_res, current(3)+z_res];...x+ y- z+
%     [current(1), current(2)+xy_res, current(3)];...%x y+ z
%     [current(1), current(2)-xy_res, current(3)];... %x y- z
%     [current(1), current(2)+xy_res, current(3)-z_res];...%x y+ z-
%     [current(1)+xy_res, current(2)-xy_res, current(3)];...% x+ y- z
%     [current(1), current(2), current(3)-z_res];... %x y z-
%     [current(1)+xy_res, current(2)+xy_res, current(3)-z_res];... x+ y+ z-
%     [current(1)+xy_res, current(2), current(3)-z_res];... %x+ y z-
%     [current(1)+xy_res, current(2)-xy_res, current(3)-z_res];...% x+ y- z-
%     [current(1), current(2)-xy_res, current(3)-z_res];... %x y- z-
%     [current(1)-xy_res, current(2), current(3)-z_res];... x- y z-
%     [current(1)-xy_res, current(2)-xy_res, current(3)-z_res];... x- y- z-
%     [current(1)-xy_res, current(2)-xy_res, current(3)]; ... x- y- z
%     [current(1)-xy_res, current(2), current(3)];...x- y z
%     [current(1)-xy_res, current(2)+xy_res, current(3)];...x- y+ z
%     [current(1)+xy_res, current(2)+xy_res, current(3)];... x+ y+ z
%     [current(1)+xy_res, current(2), current(3)];... x+ y z
%     [current(1)-xy_res, current(2)+xy_res, current(3)-z_res]]; %x- y+ z-




neighb_ind = -ones(1,size(neighbors,1));
ns = size(neighbors);
for  k = 1:ns(1)
    if(isequal(collide(map,neighbors(k,:)),0)) %if child is legal bounds, convert to subscript then index
        row = int16((neighbors(k,2)-ymin)/xy_res+1);
        col =int16((neighbors(k,1)-xmin)/xy_res+1);
        depth = int16(((neighbors(k,3)-zmin)/z_res+1));
        index = indices(col,depth,row);
        neighb_ind(k) = index;
    end
end



for  j = 1:ns(1)
    if(neighb_ind(j)~=-1)
        if(visited(neighb_ind(j))==0)
            neighb_ind(j) = -1;
        end
        
    end
    
end


end





