% MATLAB controller for Webots
% File:          	controller.m
% Date:				25-03-2013
% Description:		IVR Coursework 2
% Author:			Mark Nemec, Milan Pavlik
% Modifications:	ain't nobody got time for that

TIME_STEP = 64;
SENSOR_COUNT = 8;

distance_thresh = 600;
% 0.0125 so that (1000 - 600) * 0.0125 = 400 * 0.0125 = 5
P_GAIN = 0.05;

% no idea what this should be
I_GAIN = 0.0002;

is_turning = false;
turn_distance = 0;
errors = 0;

% Get and enable distance sensors
for i = 1 : SENSOR_COUNT
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end


% Main loop
while wb_robot_step(TIME_STEP) ~= -1

	% Obtain sensor values
	for i = 1 : SENSOR_COUNT
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
    end
	[motors_pid, errors] = pid(sensor_values(6), distance_thresh, P_GAIN, I_GAIN, errors);

    if is_turning
    	if sensor_values(5) == 0
    		is_turning = false;
    		errors = 0;
    		right_motor = clamp(-motors_pid, -10, 10);
	    	left_motor = 12 - abs(right_motor);
    	else
    		left_motor = -3;
    		right_motor = 3;
    	end
    else
    	if sensor_values(4) > 670 & sensor_values(3) > 670
    		is_turning = true;
    		turn_distance = sensor_values(4);
    		left_motor = -3;
    		right_motor = 3;
    	else
    		right_motor = clamp(-motors_pid, -10, 10);
	    	left_motor = 12 - abs(right_motor);
    	end
	end

    wb_differential_wheels_set_speed(left_motor, right_motor);

    % speeds = [left_motor, right_motor]

	drawnow;

end

% cleanup code goes here: write data to files, etc.