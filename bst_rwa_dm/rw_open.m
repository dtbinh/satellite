function ctrl = rw_open(par, dat, ctrl)

rpm2rps = 2*pi/60;

persistent cnt_p1 cnt_p2 cnt_ps p_cnt_old p1_ready cur_fw cnt_min


% Set Minimum Count
if abs(dat.spd_m)>4000*rpm2rps
    cnt_min = 10;
elseif abs(dat.spd_m)>3000*rpm2rps
    cnt_min = 7;
elseif abs(dat.spd_m)>2000*rpm2rps
    cnt_min = 4;
elseif abs(dat.spd_m)>1000*rpm2rps
    cnt_min = 2;
else
    cnt_min = 2;
end  


% Init
if(ctrl.open_trq.init)
    cnt_p1 = cnt_min;
    cnt_p2 = cnt_min;
    cnt_ps = 2*cnt_min;
    p_cnt_old = ctrl.open_trq.cnt_tot - cnt_ps;
    ctrl.open_trq.init = 0;
end
%          fprintf('tot: %d ps: %d p1:%d  p2:%d\n',ctrl.open_trq.cnt_tot,cnt_ps, cnt_p1,cnt_p2);
        if((p_cnt_old + cnt_ps) <= ctrl.open_trq.cnt_tot)
%             fprintf('-----------------cnt_ps----------------\n');

            if(dat.spd_m*ctrl.trq_tgt>0)
%                 fprintf('[%.4f] Pos Direct Control \n',tspan(i));
                ctrl.cur_cmd = par.ki*(ctrl.trq_tgt - rw_get_friction(par,dat.spd_m,'pos'));
                p1_ready = 1;   
                cnt_p1 = cnt_min;
                cnt_p2 = cnt_min;
                cnt_ps = 2*cnt_min;
                
            else
                % Calculate the Internal Current
                trq_int = rw_get_friction(par, dat.spd_m ,'neg');
            
                if(abs(trq_int) > abs(ctrl.trq_tgt))
%                     fprintf('[%.4f] Neg Parts Control trq_int:%.6f ctrl.trq_tgt:%.6f \n',tspan(i),trq_int,ctrl.trq_tgt);
                    
                    % Drive the minimum reverse current in part 1 (triggers internal current)
                    if(dat.spd_m > 0.0)  
                        ctrl.cur_cmd = -par.cur_min; 
                    else
                        ctrl.cur_cmd = +par.cur_min; 
                    end
                    
                    % Check 
                    while( ((cnt_p1/cnt_ps)*abs(trq_int)) < abs(ctrl.trq_tgt))
                        if(cnt_p2 > cnt_min)
                            cnt_p2 = cnt_p2 - 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        else
                            cnt_p1 = cnt_p1 + 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        end
                    end
                    
                    % Update cur_fw
                    cur_fw = update_cur_fw(ctrl.trq_tgt, trq_int, cnt_p1, cnt_p2, cnt_ps, dat.spd_m, par);
    
                    % Check cur_fw
                    while(abs(cur_fw) > par.cur_max)
                        if(cnt_p1 > cnt_min)
                            cnt_p1 = cnt_p1 - 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        else
                            cnt_p2 = cnt_p2 + 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        end
                    
                        cur_fw = update_cur_fw(ctrl.trq_tgt,trq_int, cnt_p1, cnt_p2, cnt_ps, dat.spd_m, par);
                    end
                    
                    p1_ready = 0;
                else
%                     fprintf('[%.4f] Neg Direct Control trq_int:%.6f ctrl.trq_tgt:%.6f \n',tspan(i),trq_int,ctrl.trq_tgt);
                    ctrl.cur_cmd = par.ki*(ctrl.trq_tgt - rw_get_friction(par,spd,'pos'));
                    p1_ready = 1;   
                    cnt_p1 = cnt_min;
                    cnt_p2 = cnt_min;
                    cnt_ps = 2*cnt_min;
                    
                end

            end
            
            % Update Count Set
            p_cnt_old = p_cnt_old + cnt_ps;
            
%          fprintf('new tot: %d ps: %d p1:%d  p2:%d\n',ctrl.open_trq.cnt_tot,cnt_ps, cnt_p1,cnt_p2);    
        elseif ((p_cnt_old + cnt_p1) <= ctrl.open_trq.cnt_tot)
          
            if (p1_ready == 0)
                
                ctrl.cur_cmd = cur_fw;
                p1_ready = 1;
            end
             
        end



end