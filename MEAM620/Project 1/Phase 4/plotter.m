
%%
%waypoints 1 - 1530
%waypoints 2 - 3270
%waypoints 3 - 1430
%waypoints 4 - 1650
%single waypoint -2800


clf
data = [trpySave(2:4,1:size(des_pos_save,2)); des_pos_save];
sz = size(ViconData,2);
clean_vd = [];
clean_data = [];

for i = 1:size(des_pos_save,2)
    %if(any(ViconData(:,i)))
        clean_vd(1:3,end+1) = ViconData(1:3,i);
        clean_data(:,end+1) = data(:,i);
    %end 
end
for i = 1:size(des_pos_save,2)
    %if(any(ViconData(:,i)))
        clean_vd(4:6,i) = ViconData(4:6,i+1530);
    %end 
end


% 
% 
% for i = 1:sz
%     if(any(data(:,i)))
%         clean_data(:,end+1) = data(:,i);
%     end 
% end



n = (size(clean_vd,2)-1)*0.1;
tl = 0:0.1:n;
ylabels = {'roll (rad)','pitch (rad)','yaw (rad)','x (m)','y (m)','z (m)'};
xlabels = {'time (s)','time (s)','time (s)','time (s)','time (s)'...
    ,'time (s)'};
figure(1);

tl = tl/10;
for i = 1:6
    title('Hover (After Takeoff)');
    subplot(6,1,i);
    hold on;
    
    plot(tl, clean_vd(i,:), 'r', 'LineWidth', 2);
     plot(tl, clean_data(i,:), 'b', 'LineWidth', 2);
    ylabel(ylabels(i));
    xlabel(xlabels(i));
    hold off;
    grid on;
    axis tight;
    %axis([0 n -2.5 2.5])
    drawnow;
end
title('Hover (After Takeoff)');


