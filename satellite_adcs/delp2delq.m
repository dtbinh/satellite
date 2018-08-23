function del_q = delp2delq(chi_sig,a,f)

del_q4 = (-a*norm(chi_sig)^2 + f*sqrt( f^2 + (1-a^2)*norm(chi_sig)^2 ))/(f^2+norm(chi_sig)^2);
del_s  = 1/f*(a + del_q4)*chi_sig;

del_q = [del_s;del_q4];

end