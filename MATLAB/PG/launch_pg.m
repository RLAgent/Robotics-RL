% Copyright 2013 The MathWorks, Inc.

% Set IP address of the host where ROS master is running.
global rosMasterIp;
rosMasterIp = 'localhost';%'158.130.108.238';

% Set IP address of the local host.
global localhostIp;
localhostIp = '127.0.1.1';

if ~exist(fullfile(matlabroot, 'toolbox', 'psp', 'rosmatlab'), 'dir')
    % Abort if ROS I/O Package is not installed.
    errordlg(['You must install ROS I/O Package before running this demo. ',...
        'Please visit www.mathworks.com/ros to download ROS I/O Package.'], ...
        'ROS I/O Package Not Found');
else
    % Launch Turtlebot Controller Demo.
    turtlebot_control;
end