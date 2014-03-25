function [out, err] = pid(pv, setpoint, p_gain, i_gain, err)
	p = (setpoint - pv) * p_gain;
	ii = (setpoint - pv) * i_gain;
	err = err + ii;
	out = p + ii;
end