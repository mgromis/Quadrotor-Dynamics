%% quad_traj_starter.m
% 
% This Matlab script provides the starter code for the quadrotor trajectory
% problem on Homework 1 in MEAM 620.
% This script is based on the flying_box_starter.m file original written by 
% Professor Katherine J. Kuchenbecker in September of 2012.  It was
% modified by Philip Dames in January of 2016.
% 
%% SETUP

% Delete all variables from our workspace.
clear

% Load the Vicon data recorded during the movie.
% This MATLAB data file includes time histories of the x, y, and z
% coordinates in meters, as well as time histories of the Euler angles
% in radians.
load quad_trajectory;

% Open figure 1 and clear it to get ready for plotting.
figure(1)
clf

%% DEFINITIONS

% We need to keep track of two frames in this code.
%
% Frame 0 is the frame of the Vicon system, which is shown in the handout.
% The is the base frame, and it's what we plot in.
%
% Frame 1 is the frame of the quadrotor, which is flying around. The frame 
% is intially oriented in the same manner as the Vicon frame. Its x-axis 
% is straight out along the arm with the red tape.  Its z-axis is out of 
% the top, and its y-axis is originally to the left.

% Define the locations of the four rotors in the quadrotor frame.
% We include a 1 at the end of each of these column vectors for use 
% with homogenous transformations. 
diam = 0.4;  % meters
pa1 = [ diam/2;       0; 0; 1]; % rotor along the positive x-axis
pb1 = [-diam/2;       0; 0; 1]; % rotor along the negative x-axis
pc1 = [      0;  diam/2; 0; 1]; % rotor along the positive y-axis
pd1 = [	     0; -diam/2; 0; 1]; % rotor along the negative y-axis
pe1 = [      0;       0; 0; 1]; % center

%% ANIMATION
% Play back the recorded motion of the quadrotor.

% You may not use any built-in or downloaded functions dealing with
% rotation matrices, homogeneous transformations, Euler angles,
% roll/pitch/yaw angles, or related topics.  Instead, you must type all
% your calculations yourself, usin only low-level functions such as
% sin, cos, and vector/matrix math. 

% Do your animation.  All of your code should be between the two lines of stars.
% *******************************************************************

% You will need to parse the data file to extract the pose history of the
% quadrotor.  Use this history to create transformations from the Vicon
% frame to the quadrotor frame using your function data2transformation.
% Using these transformations, you will need to transform all of the points, 
% pa1, pb1, pc1, pd1, pe1 from the quadrotor frame (frame 1) to the Vicon 
% frame (frame 0).  Use the provided plotQuad function to animate the 
% resulting trajectory.
plotdelay = .001;

size = size(data);
size = size(2);
time = data(1, :);
x1 = data(2, :);
y1 = data(3, :);
z1 = data(4, :);
phi1 = data(5, :);
theta1 = data(6, :);
psi1 = data(7, :);


for i = 1:size
    Transformation = data2transformation(x1(i), y1(i), z1(i), phi1(i),...
        theta1(i), psi1(i));
    pa = Transformation*pa1;
    pb = Transformation*pb1;
    pc = Transformation*pc1;
    pd = Transformation*pd1;
    pe = Transformation*pe1;
    if i==1
         h = plot_quad(pa, pb,pc, pd,pe);
    else
        h = plot_quad(pa, pb,pc, pd,pe, h);
        
    end
    pause(plotdelay);
end   
         
    

% *******************************************************************

