function [C] = out_of_bounds(map, points)

if(isequal(points,zeros(0,3)))
    C = [];
end


if(isequal(map,zeros(0,3)))
    s = size(points);
    C = ones(s(1),1);
    return
end

xmin = map(1,10);
ymin =map(1,11);
zmin = map(1,12);
xmax = map(2,10);
ymax = map(2,11);
zmax = map(2,12);

xy_res = map(3,10);
z_res = map(3,11);
margin = map(3,12);
sizemap = size(map);
sizepoints = size(points);
C = zeros(sizepoints(1),1);
for i =1:sizepoints(1)
    for k=1:sizemap(1)
        
    hit_boarder = points(i,1)<xmin||points(i,1)>xmax||points(i,2)<ymin||points(i,2)>ymax...
        ||points(i,3)<zmin||points(i,3)>zmax;
    if(hit_boarder) 
    C(i,1) = 1;
    break 
    end
      
    end
end
end




