% MATLAB controller for Webots
% File:          	controller.m
% Date:				25-03-2013
% Description:		IVR Coursework 2
% Author:			Mark Nemec, Milan Pavlik
% Modifications:	ain't nobody got time for that

TIME_STEP = 64;
SENSOR_COUNT = 8;
% DISTANCE_THRESH = 600;
P_GAIN = 0.01;
I_GAIN = 0.01;

RATIO = 5;
errors = [0 0 0 0 0 0 0 0];
distance_thresh = [0, 0, 0, 0, 500, 700, 0, 0];

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
    	[motors_pid(i), errors(i)] = pid(sensor_values(i), distance_thresh(i), P_GAIN, I_GAIN, errors(i));
    end

    front_distance = sensor_values(4);

    left_motor = 0;
    right_motor = 0;

    % left_motor = (motors_pid(3) + motors_pid(4)) / 2.0;
    left_motor = motors_pid(4);
    right_motor = - motors_pid(6) * SPEED_FACTOR;

	left_motor = clamp(SPEED_FACTOR + (RATIO * motors_pid(4)), -5, 5);
	right_motor = - (motors_pid(5) + 2 * motors_pid(6)) * SPEED_FACTOR;

    wb_differential_wheels_set_speed(left_motor, right_motor);

    % disp(sensor_values);

    % disp(left_motor);
    % disp(right_motor);








	% read the sensors, e.g.:
	%  rgb = wb_camera_get_image(camera);

	% Process here sensor data, images, etc.

	% send actuator commands, e.g.:
	%  wb_differential_wheels_set_speed(500, -500);

	% if your code plots some graphics, it needs to flushed like this:
	drawnow;

end