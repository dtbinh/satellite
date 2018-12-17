function deriv = fncA(x)
m = 50;
g = 9.81;
l = 0.1;

deriv      = zeros(3,1);
deriv(1,1) = x(2,1);
deriv(2,1) = -(m*g*l*sin(x(1)))/(x(3)+m*l^2);

end