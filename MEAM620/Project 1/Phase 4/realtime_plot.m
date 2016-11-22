function realtime_plot( t, ViconData, trpySave, DesPosSave )
%REALTIME_PLOT Summary of this function goes here
%   Detailed explanation goes here

data = [trpySave(2:4,:); DesPosSave];
sz = size(ViconData,2);
if sz < 21
    return;
end

n = 20;
tl = 1:1:(2*n);
ts = tl + t - n;
figure(1);
for i = 1:6
    hold on;
    subplot(6,1,i);
    plot(ts, ViconData(i,end-n:end), 'r', 'LineWidth', 2);
    plot(ts, data(i,end-n:end), 'b', 'LineWidth', 2);
    
    plot(t, ViconData(i,end), 'r.', 'LineWidth', 5);
    plot(t, data(i,end), 'b.', 'LineWidth', 5);
    
    hold off;
    grid on;
    axis tight;
    drawnow;
end

