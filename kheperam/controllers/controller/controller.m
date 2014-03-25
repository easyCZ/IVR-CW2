% MATLAB controller for Webots
% File:          	controller.m
% Date:				25-03-2013
% Description:		IVR Coursework 2
% Author:			Mark Nemec, Milan Pavlik
% Modifications:	ain't nobody got time for that

TIME_STEP = 64;
SENSOR_COUNT = 8;
% DISTANCE_THRESH = 600;
GAIN = 0.016666666666666666;

distance_thresh = [0, 0, 200, 200, 300, 700, 0, 0];
weights = [0, 0, 0, 7, 0, 2, 0, 0];
errors = [0, 0, 0, 0, 0, 0, 0, 0];

SPEED_FACTOR = 5;


% Get and enable distance sensors
for i = 1 : SENSOR_COUNT
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end


% Main loop
while wb_robot_step(TIME_STEP) ~= -1

	% Obtain sensor values
   	for i=1: SENSOR_COUNT
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
    end

    for i = 1: SENSOR_COUNT
    	[motors_pid(i), errors(i)] = pid(sensor_values(i), distance_thresh(i), GAIN, errors(i));
    end

    front_distance = sensor_values(3);

    left_motor = 0;
    right_motor = 0;

    front_distance = sensor_values(4);

    if front_distance > 0
    	left_motor = -3;
    	right_motor = 3;
    else
    	left_motor = motors_pid(4);
	    right_motor = - motors_pid(6) * SPEED_FACTOR;
    end

    % left_motor = (motors_pid(3) + motors_pid(4)) / 2.0;


    % if front_distance > 600
    % 	left_motor = -SPEED_FACTOR;
    % 	right_motor = SPEED_FACTOR;
    % else
    % 	left_motor = 1 * SPEED_FACTOR;
    % 	right_motor = -(motors_pid(5) + 2 * motors_pid(6)) * SPEED_FACTOR;
    % end

    wb_differential_wheels_set_speed(left_motor, right_motor);

    % disp(sensor_values);

    % speeds = [left_motor, right_motor]

    % disp(speeds);
    % disp(right_motor);








	% read the sensors, e.g.:
	%  rgb = wb_camera_get_image(camera);

	% Process here sensor data, images, etc.

	% send actuator commands, e.g.:
	%  wb_differential_wheels_set_speed(500, -500);

	% if your code plots some graphics, it needs to flushed like this:
	drawnow;

end

% cleanup code goes here: write data to files, etc.