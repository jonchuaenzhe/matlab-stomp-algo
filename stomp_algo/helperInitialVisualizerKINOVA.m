% Copyright 2020 The MathWorks, Inc.

% Initial visualizer
positions = x0(1:numJoints)';

% figure(): The 1st and 2nd number is the pixel position of the figure relative to
% the monitor display. The 3rd and 4th number are the width and height of
% the figure.
hgif = figure('Position', [375 446 641 480]);
% 'PreservePlot'= True: The function does not overwrite previous plots displayed by calling show. 
% This setting functions similarly to hold on for a standard MATLAB® figure, 
% but is limited to robot body frames. When you specify this argument as 
% false, the function overwrites previous plots of the robot.
% 'Frames': Display body frames
ax1 = show(robot, positions(:,1),'PreservePlot', false, 'Frames', 'on', 'Visuals', 'on');
% view(az,el) sets the azimuth and elevation angles of the camera's line of
% sight for the current axes
view(150,29)
hold on
% set the x,y,z, data limits of the figure
axis([-0.8 0.8 -0.6 0.7 -0.2 0.7]);
% plot the final position
plot3(poseFinal(1), poseFinal(2), poseFinal(3),'r.','MarkerSize',20)

% Visualize collision world
for i=1:numel(world)
    [~,pObj] = show(world{i});
    pObj.LineStyle = 'none';
end




