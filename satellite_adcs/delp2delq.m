function dq = delp2delq(dp,a,f)

dq4 = (-a*norm(dp)^2 + f*sqrt( f^2 + (1-a^2)*norm(dp)^2 ))/(f^2+norm(dp)^2);
dq13  = (f^-1)*(a + dq4)*dp;

dq = [dq13;dq4];

end