function h = plot_quad(pa0, pb0, pc0, pd0, pe0, h)
% plot_quad plots the quadrotor.  The positive x axis is colored red to
% match the quadrotor from the video.  The final input h is optional, but
% but recommended using it if you call this function in a loop.  For example,
% the first time through the loop you would call
% h = plotQuad(pa0, pb0, pc0, pd0, pe0);.  All subsequent iterations through
% the loop would use h = plotQuad(pa0, pb0, pc0, pd0, pe0, h); where you
% feed in the h from the previous iteration into the current call.  This
% will significantly speed up the plotting.
% 
% This function is based on the flying_box_starter.m file original written by 
% Professor Katherine J. Kuchenbecker in September of 2012.  It was
% modified by Philip Dames in January of 2016.

if nargin == 5

    % Plot the arms of the quadrotor, with the arm oriented along the 
    % positive x-axis in the body frame colored red.
    h = plot3([pe0(1) pa0(1)], [pe0(2) pa0(2)], [pe0(3) pa0(3)], 'r', ...
        [pe0(1) pb0(1)], [pe0(2) pb0(2)], [pe0(3) pb0(3)], 'b', ...
        [pc0(1) pd0(1)], [pc0(2) pd0(2)], [pc0(3) pd0(3)], 'b', 'LineWidth', 2);

    % Enforce that one unit is displayed equivalently in x, y, and z.
    axis equal;

    % Set the viewing volume to the values known to be correct.  If
    % you cannot see anything in your plot, you should increase the
    % ranges here or comment this out.
    axis([-2 2 -2 2 -0 2.5])

    % Set the viewing angle to be similar to the camera view in the video.
    view(-135, 40)

    % Label the axes.
    xlabel('x0 (m)')
    ylabel('y0 (m)')
    zlabel('z0 (m)')

    % Turn on the grid to make it easy to see the walls.
    grid on

    title('Quadrotor Trajectory')

else

    % Set the locations of the rotors to the current points.  Using
    % set and the plot handle in this way is faster than replotting
    % everything.
    set(h(1),'xdata',[pe0(1) pa0(1)],'ydata',[pe0(2) pa0(2)],'zdata',[pe0(3) pa0(3)])
    set(h(2),'xdata',[pe0(1) pb0(1)],'ydata',[pe0(2) pb0(2)],'zdata',[pe0(3) pb0(3)])
    set(h(3),'xdata',[pc0(1) pd0(1)],'ydata',[pc0(2) pd0(2)],'zdata',[pc0(3) pd0(3)])

end
