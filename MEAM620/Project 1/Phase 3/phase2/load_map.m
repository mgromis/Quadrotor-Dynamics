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


ind_bound = 0;
block_ind = [];
for i= 1:length(lines{1})
    if strcmp(lines{1}{i}, 'block')
        block_ind = [block_ind; i];
    end
     if strcmp(lines{1}{i}, 'boundary')
        boundary_ind = i;
    end
end
xmin = str2double(lines{1}{boundary_ind+1});
ymin = str2double(lines{1}{boundary_ind+2});
zmin = str2double(lines{1}{boundary_ind+3});
xmax = str2double(lines{1}{boundary_ind+4});
ymax = str2double(lines{1}{boundary_ind+5});
zmax = str2double(lines{1}{boundary_ind+6});


bound = zeros(length(block_ind),9);
for i = 1:length(block_ind)
    ind = block_ind(i);
        bound(i,1) = str2double(lines{1}{ind+1});
        bound(i,2)= str2double(lines{1}{ind+2});
        bound(i,3)= str2double(lines{1}{ind+3});
        bound(i,4)= str2double(lines{1}{ind+4});
        bound(i,5) = str2double(lines{1}{ind+5});
        bound(i,6) = str2double(lines{1}{ind+6});
        bound(i,7) = str2double(lines{1}{ind+7});
        bound(i,8) = str2double(lines{1}{ind+8});
        bound(i,9) = str2double(lines{1}{ind+9});
end

    

    map.obst = bound;
    map.data = [xmin ymin zmin xmax ymax zmax xy_res z_res];
    map.margin = margin;








