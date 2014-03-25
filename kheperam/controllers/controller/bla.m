% MATLAB controller for Webots
% File:          	controller.m
% Date:				25-03-2013
% Description:		IVR Coursework 2
% Author:			Mark Nemec, Milan Pavlik
% Modifications:	ain't nobody got time for that

TIME_STEP = 64;
SENSOR_COUNT = 8;
% DISTANCE_THRESH = 600;
P_GAIN = 0.001;
I_GAIN = 0.0005;

TURN_THRESH = 5;


errors = [0 0 0 0 0 0 0 0];
distance_thresh = [0, 0, 0, 0, 300, 600, 0, 0];

SPEED_FACTOR = 10;

is_turning = 0;
turn_distance = 600;
found_wall = 0;



% Get and enable distance sensors
for i = 1 : SENSOR_COUNT
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end


% Main loop
while wb_robot_step(TIME_STEP) ~= -1

	% Obtain sensor values
   	for i = 1: SENSOR_COUNT
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
    	[motors_pid(i), errors(i)] = pid(sensor_values(i), distance_thresh(i), P_GAIN, I_GAIN, errors(i));
    end

    % left_motor = (motors_pid(3) + motors_pid(4)) / 2.0;

    if is_turning
    	if abs(sensor_values(6) - turn_distance) < TURN_THRESH
    		is_turning = 0;
    		left_motor = motors_pid(4);
	    	right_motor = - motors_pid(6) * SPEED_FACTOR;
    	else
    		left_motor = -3;
    		right_motor = 3;
    	end

    else
    	if sensor_values(4) > 700
    		is_turning = 1;
    		left_motor = -3;
    		right_motor = 3;
    	else
    		left_motor = motors_pid(4);
	    	right_motor = - motors_pid(6) * SPEED_FACTOR;
    	end
	end

    wb_differential_wheels_set_speed(left_motor, right_motor);

    % speeds = [left_motor, right_motor]

	drawnow;

end

% cleanup code goes here: write data to files, etc.