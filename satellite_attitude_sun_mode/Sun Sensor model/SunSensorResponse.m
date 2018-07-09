function meas_angle = SunSensorResponse(true_angle)
% SunSensorResponse. Implements the sun sensor response shown in the P6 notes.

true_angle_max = 83*pi/180;
meas_angle_max = 0.5*(sin(7*pi/180) + cos(7*pi/180)*tan(true_angle_max))./(1+tan(true_angle_max).^2).^0.5;


if abs(true_angle)<7*pi/180
	meas_angle = true_angle;
elseif true_angle>0&true_angle<83*pi/180
    meas_angle = 0.5*(sin(7*pi/180) + cos(7*pi/180)*tan(true_angle))./(1+tan(true_angle).^2).^0.5;
elseif true_angle<0&true_angle>-83*pi/180
    meas_angle = -0.5*(sin(7*pi/180) - cos(7*pi/180)*tan(true_angle))./(1+tan(true_angle).^2).^0.5;
elseif true_angle>=83*pi/180
    meas_angle = -meas_angle_max/(7*pi/180)*(true_angle-pi/2);
elseif true_angle<=-83*pi/180
    meas_angle = -meas_angle_max/(7*pi/180)*(true_angle+pi/2);
end
	
