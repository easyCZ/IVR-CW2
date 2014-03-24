% MATLAB controller for Webots
% File:          	controller.m
% Date:				25-03-2013
% Description:		IVR Coursework 2
% Author:			Mark Nemec, Milan Pavlik
% Modifications:	ain't nobody got time for that

TIME_STEP = 64;
SENSOR_COUNT = 8;

% Get and enable distance sensors
for i = 1 : SENSOR_COUNT
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end


% Main loop
while wb_robot_step(TIME_STEP) ~= -1

	% Obtain sensor values
   	for i=1:N
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
    end

  	
    
    


	% read the sensors, e.g.:
	%  rgb = wb_camera_get_image(camera);

	% Process here sensor data, images, etc.

	% send actuator commands, e.g.:
	%  wb_differential_wheels_set_speed(500, -500);

	% if your code plots some graphics, it needs to flushed like this:
	drawnow;

end

% cleanup code goes here: write data to files, etc.
