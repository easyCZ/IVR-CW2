function [err] = pid(pv, setpoint, gain)
	err = (setpoint - pv) * gain;
end