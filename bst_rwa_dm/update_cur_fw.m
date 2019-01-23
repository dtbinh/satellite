function cur_fw = update_cur_fw(trq_tgt,trq_int, cnt_p1, cnt_p2, cnt_ps, ki, frct_p, spd_m)

cur_fw = cnt_ps*ki*trq_tgt - cnt_p1*ki*trq_int;
cur_fw = cur_fw/cnt_p2;
cur_fw = cur_fw - ki*polyval(frct_p,spd_m);

end