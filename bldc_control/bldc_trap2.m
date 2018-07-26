function out = bldc_trap2(in)
theta_e = in;

%%
if ((theta_e >= 0) && (theta_e < 2*pi/3))
    out = 1; fprintf('1\n');
else
    if ((theta_e >= 2*pi/3) && (theta_e < pi))
        out = 1 - 6/pi*(theta_e-2*pi/3);
        fprintf('2\n');
    else
        if ((theta_e >= pi) && (theta_e < 5*pi/3))
            out = -1;fprintf('3\n');
        else
            if ((theta_e >= 5*pi/3) && (theta_e < 2*pi))
                out = -1 + 6/pi*(theta_e-5*pi/3);fprintf('4\n');
            else
                fprintf('error');
            end
        end
    end
end

end
             