function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, i) touches an obstacle and is 0 otherwise.
xmin = map.data(1);
ymin =map.data(2);
zmin = map.data(3);
xmax = map.data(4);
ymax = map.data(5);
zmax = map.data(6);
margin = map.margin;

psize = size(points);
C = zeros(psize(1),1);
min = [xmin ymin zmin];
max = [xmax ymax zmax];


obst_size = size(map.obst);
hitobst=0;
for i =1:psize(1)
    hit_boarder = points(i,1)<xmin||points(i,1)>xmax||points(i,2)<ymin||points(i,2)>ymax...
        ||points(i,3)<zmin||points(i,3)>zmax;
    if(hit_boarder)
        C(i,1) = 1;
    else
        
    for k=1:obst_size(1)
        
    
       hitobst = points(i,1)>=(map.obst(k,1)-margin)&&...
        points(i,1)<=(map.obst(k,4)+margin)&&points(i,2)>=(map.obst(k,2)-margin)&&...
        points(i,2)<=(map.obst(k,5)+margin)&&points(i,3)>=(map.obst(k,3)-margin/2.0)&&...
        points(i,3)<=(map.obst(k,6)+margin/2.0);
        if(hitobst)
            
    C(i,1) = 1;
            break
        end
      
        end
    end
end
end

    
