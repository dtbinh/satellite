function out = bldc_trap(theta)
if theta < 0
theta = theta + 2*pi;
end
if theta > 2*pi
theta = theta - 2*pi;
end

if(theta>=0)&&(theta<2*pi/3)
    out = 1;
else if (theta>=2*pi/3)&&(theta<pi)
        out = 1-6/pi*(theta-2*pi/3);
    else if (theta>=pi)&&(theta<5*pi/3)
            out = -1;
        else if (theta>=5*pi/3)&&(theta<=2*pi)
                out = -1+6/pi*(theta-5*pi/3);
            else
                fprintf('error\n');
            end
        end
    end
end
end