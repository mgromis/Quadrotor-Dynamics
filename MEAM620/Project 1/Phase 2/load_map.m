function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

if isempty(importdata(filename))
    map = zeros(0,3);
    return
end
id = fopen(filename);
lines = textscan(id,'%s');
fclose(id);
xmin = 0;
ymin = 0;
zmin = 0;
xmax = 0;
ymax =0;
zmax = 0;
items = cellfun('length',lines);
items_temp = items;
num_blocks = int16((items-7)/10);

    


bound = zeros(num_blocks,9);
linecounter = 0;


for i=1:(num_blocks+1)
    if(strcmp(lines{1}{i},'boundary'))
            xmin = str2double(lines{1}{2});
ymin = str2double(lines{1}{3});
zmin = str2double(lines{1}{4});
xmax = str2double(lines{1}{5});
ymax = str2double(lines{1}{6});
zmax = str2double(lines{1}{7});
linecounter = linecounter+7;
    else 
        bound(i-1,1) = str2double(lines{1}{linecounter+2});
        bound(i-1,2)= str2double(lines{1}{linecounter+3});
        bound(i-1,3)= str2double(lines{1}{linecounter+4});
        bound(i-1,4)= str2double(lines{1}{linecounter+5});
        bound(i-1,5) = str2double(lines{1}{linecounter+6});
        bound(i-1,6) = str2double(lines{1}{linecounter+7});
        bound(i-1,7) = str2double(lines{1}{linecounter+8});
        bound(i-1,8) = str2double(lines{1}{linecounter+9});
        bound(i-1,9) = str2double(lines{1}{linecounter+10});
        linecounter = linecounter+10;
    end
end

    map.obst = bound;
    map.data = [xmin ymin zmin xmax ymax zmax xy_res z_res];
    map.margin = margin;








