function [ short_path ] = safe_short_path( map, path )

path_qn = path;
N = size(path_qn,1);
p = path_qn(1,:);

short_path = p;
done = false;

i = 1;
while i < N
    
    for j = (i+1):N,
        line_tics = 2*(j-i);
        line_path = [linspace(p(1), path_qn(j,1), line_tics)', ...
            linspace(p(2), path_qn(j,2), line_tics)', ...
            linspace(p(3), path_qn(j,3), line_tics)'];
        if any(collide(map,line_path)),
            short_path = [short_path; path_qn((j-1),:)];
            i = j-1;
            break;
        end
        if j == N
            done = true;
        end
    end
    p = path_qn(i,:);
    if done
        short_path = [short_path; path_qn(end,:)];
        break;
    end
end


end

