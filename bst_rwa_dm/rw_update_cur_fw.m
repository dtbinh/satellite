function cur_fw = rw_update_cur_fw(trq_tgt, trq_int, cnt_p1, cnt_p2, cnt_ps, spd, RW)

cur_fw = cnt_ps*RW.ki*trq_tgt - cnt_p1*RW.ki*trq_int;
cur_fw = cur_fw/cnt_p2;
cur_fw = cur_fw - RW.ki*rw_get_friction(RW,spd,'pos');

end