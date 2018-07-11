function  output = sun_sensor(input)
s_b        = input(1:3);
model_mode = input(4);
% SunSensor. Models Sun sensor by converting input sun vector (in body
% frame) to Sun sensor [alpha, beta] angles.
%
% Format:
%   [alpha, beta] = SunSensor(s_b, model_mode)
% where:
%   u           is the sun vector in the body frame
%   model_mode  selects ideal sensor (=1) or non-linear sensor (=2)
%   angles      is [alpha, beta] angles measured by the sun sensor
% Uses the method described in section 4.1 of P6 notes

% calculate [alpha, beta] angles - note this algorithm only works when the
% sun vector lies within 90deg of the sensor boresight (=Z axis - see
% figure 3-8 of P4 notes)

alpha = atan2(+s_b(2),s_b(3));
beta  = atan2(-s_b(1),s_b(3));

% apply non-linear characteristic if requested by user input
if model_mode==2
    alpha = SunSensorResponse(alpha);
    beta  = SunSensorResponse(beta);
end

output = [alpha, beta];

return

% =========================================================================
% Private functions (can only be called from the SunSensor function
% =========================================================================
function meas_angle = SunSensorResponse(true_angle)
% SunSensorResponse. Implements the sun sensor response shown in lecture notes.
meas_angle     = 0;
true_angle_max = 83*pi/180;
meas_angle_max = 0.5*(sin(7*pi/180) + cos(7*pi/180)*tan(true_angle_max))./(1+tan(true_angle_max).^2).^0.5;

if abs(true_angle)<7*pi/180
	meas_angle = true_angle;
elseif true_angle>0&&true_angle<83*pi/180
    meas_angle = 0.5*(sin(7*pi/180) + cos(7*pi/180)*tan(true_angle))./(1+tan(true_angle).^2).^0.5;
elseif true_angle<0&&true_angle>-83*pi/180
    meas_angle = -0.5*(sin(7*pi/180) - cos(7*pi/180)*tan(true_angle))./(1+tan(true_angle).^2).^0.5;
elseif true_angle>=83*pi/180
    meas_angle = -meas_angle_max/(7*pi/180)*(true_angle-pi/2);
elseif true_angle<=-83*pi/180
    meas_angle = -meas_angle_max/(7*pi/180)*(true_angle+pi/2);
    

end

