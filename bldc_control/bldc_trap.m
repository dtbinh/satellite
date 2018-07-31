function out = bldc_trap(in,type)
if ~exist('type','var')
    type = 'cosine';
end

switch type
    case 'cosine'
        out = cos(in);
        out = 1/0.5*sat(out,0.5);

    case 'trap'
        
        if ((in >= 0) && (in < 2*pi/3))
            out = 1; fprintf('1\n');
        else
            if ((in >= 2*pi/3) && (in < pi))
                out = 1 - 6/pi*(in-2*pi/3);
                fprintf('2\n');
            else
                if ((in >= pi) && (in < 5*pi/3))
                    out = -1;fprintf('3\n');
                else
                    if ((in >= 5*pi/3) && (in < 2*pi))
                        out = -1 + 6/pi*(in-5*pi/3);fprintf('4\n');
                    else
                        fprintf('error');
                    end
                end
            end
        end
        
    otherwise
end



end