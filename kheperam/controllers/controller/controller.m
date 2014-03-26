% MATLAB controller for Webots
% File:             controller.m
% Date:             25-03-2013
% Description:      IVR Coursework 2
% Author:           Mark Nemec, Milan Pavlik
% Modifications:    ain't nobody got time for that

TIME_STEP = 64;
SENSOR_COUNT = 8;
WALL_THRESH = 670;
DISTANCE_THRESH = 600;
TURN_STOP_THRESH = 400;
% 0.0125 so that (1000 - 600) * 0.0125 = 400 * 0.0125 = 5
P_GAIN = 0.05;

% this value worked well experimentally
I_GAIN = 0.0002;

% boolean flags used for flow control
is_turning = false;

errors = 0;

% Get and enable distance sensors
for i = 1 : SENSOR_COUNT
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i), TIME_STEP);
end

% Main loop
while wb_robot_step(TIME_STEP) ~= -1

    % Obtain sensor values
    for i = 1 : SENSOR_COUNT
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
    end

    % apply PID Controller to the rightmost sensor
    [motors_pid, errors] = pid(sensor_values(6), DISTANCE_THRESH, P_GAIN, I_GAIN, errors);

    if is_turning
        if sensor_values(5) <= TURN_STOP_THRESH
            % stop once 5th sensor is less than the threshold
            is_turning = false;
            % reset errors of I-component
            errors = 0;
            % clamp value between -10 and 10
            vright = clamp(-motors_pid, -10, 10);
            % Balance 12 between left and right motor so that we
            % don't get to a point where one motor is 10 and the other is 0
            vleft = 12 - abs(vright);
            % Override left when right becomes extremely small
            if vright < -9
                vleft = 10;
            end
        else
            % slow turn when we hit the wall
            vleft = -3;
            vright = 3;
        end
    else
        if sensor_values(4) > WALL_THRESH & sensor_values(3) > WALL_THRESH
            is_turning = true;
            % slow turn when we hit the wall
            vleft = -3;
            vright = 3;
        else
            % clamp value between -10 and 10
            vright = clamp(-motors_pid, -10, 10);
            % Balance 12 between left and right motor so that we
            % don't get to a point where one motor is 10 and the other is 0
            vleft = 12 - abs(vright);
            % Override left when right becomes extremely small
            if vright < -9
                vleft = 10;
            end
        end
    end

    wb_differential_wheels_set_speed(vleft, vright);
    drawnow;
end
