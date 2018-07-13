function output = normalquat(input)
q      = input(1:4);

q = q./norm(q);

output = q;

end