function out = rw_satzero(in,check,max)

if abs(check)< abs(max)
    out = in;
else
%     fprintf('saturation\n');
    out = 0;
end

end