% This function calculates the trq using different parts in the open torque
% control mode.
% assuming explicit current driven is smaller than actual int. current
% (so only int. current flows through motor ...)
% 
% tgt    = ( num1*trq_int + num2*trq_fw ) / nums                           (average torque)
% trq_fw = ( nums*tgt - num1*trq_int    ) / num2
%           with trq_int = bst_mdl_rw_get_friction(NEG);
%               trq_fw  = km * cur_fw + bst_mdl_rw_get_friction(POS);
% cur_fw =   ( ( nums*tgt*ki - num1*trq_int*ki ) / num2 ) - bst_mdl_rw_get_friction(POS)*ki
%        = ( ( ( nums*tgt    - num1*trq_int    ) / num2 ) - bst_mdl_rw_get_friction(POS)    ) * ki    

function cur_fw = rw_update_cur_fw(trq_tgt, trq_int, cnt_p1, cnt_p2, cnt_ps, spd, temp, par)

trq = (cnt_ps*trq_tgt - cnt_p1*trq_int)/cnt_p2;
trq = trq - rw_get_friction(par,spd,temp,'pos');
cur_fw = trq/par.km_nom;

cur_fw = trq/rw_get_km(par, spd, cur_fw);

end