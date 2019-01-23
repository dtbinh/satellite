function out = satcheck(in,max)

if abs(in)< abs(max)
    out = in;
else
%     fprintf('saturation\n');
    out = sign(in)*abs(max);
end

end