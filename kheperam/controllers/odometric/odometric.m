% MATLAB controller for Webots
% File:          	controller.m
% Date:				25-03-2013
% Description:		IVR Coursework 2
% Author:			Mark Nemec, Milan Pavlik
% Modifications:	ain't nobody got time for that

TIME_STEP = 64;
SENSOR_COUNT = 8;
ROBOT_RADIUS = 53 / 2.0;
WHEEL_RADIUS = 8;

distance_thresh = 600;
% 0.0125 so that (1000 - 600) * 0.0125 = 400 * 0.0125 = 5
P_GAIN = 0.05;

% no idea what this should be
I_GAIN = 0.0002;

is_turning = false;
ready_to_stop = false;
errors = 0;

x = 0; y = 0; theta = 0;

% Get and enable distance sensors
for i = 1 : SENSOR_COUNT
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i), TIME_STEP);
end

% Enable the wheel encoders
wb_differential_wheels_enable_encoders(TIME_STEP);

% Main loop
while wb_robot_step(TIME_STEP) ~= -1

    % Obtain sensor values
    for i = 1 : SENSOR_COUNT
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
    end

    % apply PID Controller to the rightmost sensor
    [motors_pid, errors] = pid(sensor_values(6), distance_thresh, P_GAIN, I_GAIN, errors);

    % get encoder values
    encoder_values = [wb_differential_wheels_get_left_encoder() wb_differential_wheels_get_right_encoder()];
    % get the number of revolutions
    encoder_values = encoder_values / (2 * 100.0 * pi);
    % and then distance in millimeters
    encoder_values = encoder_values * 2 * pi * WHEEL_RADIUS;
    % reset encoders
    wb_differential_wheels_set_encoders(0, 0);

    if is_turning
    	if sensor_values(5) <= 400
    		is_turning = false;
    		errors = 0;
    		vright = clamp(-motors_pid, -10, 10);
	    	vleft = 12 - abs(vright);
    		if vright < -9
    			vleft = 10
    		end
    	else
    		vleft = -3;
    		vright = 3;
    	end
    else
    	if sensor_values(4) > 670 & sensor_values(3) > 670
    		is_turning = true;
    		vleft = -3;
    		vright = 3;
    	else
    		vright = clamp(-motors_pid, -10, 10);
	    	vleft = 12 - abs(vright);
    		if vright < -9
    			vleft = 10
    		end
    	end
	end

    % Odometry shit
    x = x + 0.5 * (encoder_values(1) + encoder_values(2)) * cos(theta);
    y = y + 0.5 * (encoder_values(1) + encoder_values(2)) * sin(theta);
	theta = theta - 0.5 * (encoder_values(1) - encoder_values(2)) / (ROBOT_RADIUS);
    disp([x y theta]);

    if abs(x) < 3 & abs(y) < 3 & ready_to_stop
        % Back to the start
        wb_differential_wheels_set_speed(0, 0);
        % do one more step and break out
        wb_robot_step(TIME_STEP);
        break;
    else
        if abs(x) > 10 & abs(y) > 10
            % We left the nearby area of the start
            ready_to_stop = true;
        end
        wb_differential_wheels_set_speed(vleft, vright);
    end

	drawnow;

end

% cleanup code goes here: write data to files, etc.