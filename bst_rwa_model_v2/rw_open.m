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
%   fprintf('tot: %d ps: %d p1:%d  p2:%d\n',ctrl.open_trq.cnt_tot,cnt_ps, cnt_p1,cnt_p2);



if((p_cnt_old + cnt_ps) <= ctrl.open_trq.cnt_tot)
fprintf('-----------------cnt_ps----------------\n');

    frctn_p = rw_get_friction(par, dat.spd_m, dat.temp, 'pos');

    if( (dat.spd_m*ctrl.trq_tgt >= 0.0) || ...
       ((dat.spd_m*ctrl.trq_tgt <  0.0) && (abs(ctrl.trq_tgt) < abs(frctn_p))) )
   
        
        trq = ctrl.trq_tgt - frctn_p;
        ctrl.cur_cmd = trq/par.km_nom;
        km = rw_get_km(par, dat.spd_m, ctrl.cur_cmd);
        
        ctrl.cur_cmd = trq/km;
        p1_ready = 1;   
        cnt_p1 = cnt_min;
        cnt_p2 = cnt_min;
        cnt_ps = 2*cnt_min;
        fprintf('Pos Direct Control p1:p1=%d p2=%d\n',cnt_p1,cnt_p2);

    else
         
        trq = ctrl.trq_tgt - frctn_p;
        cur_stgt = trq/par.km_nom;
        km = rw_get_km(par, dat.spd_m, cur_stgt);
        cur_stgt = trq/km;
        
        % Calculate the Internal Current
        cur_int = rw_get_cur_int(par, dat.spd_m, cur_stgt, dat.temp, 0);

        fprintf('Neg Operation cur_int%9.6f\n',cur_int);
        if(cur_int~=0.0)
%         fprintf('Neg Parts Control trq_int:%.6f ctrl.trq_tgt:%.6f \n',trq_int,ctrl.trq_tgt);
            
            frctn_int = km*cur_int + frctn_p;
            fprintf('frctn_int %9.6f\n',frctn_int);
            % Drive the minimum reverse current in part 1 (triggers internal current)
            if(dat.spd_m > 0.0)  
                ctrl.cur_cmd = -par.cur_min; 
            else
                ctrl.cur_cmd = +par.cur_min; 
            end

            % Check 
            while( ((cnt_p1/cnt_ps)*abs(frctn_int)) < abs(ctrl.trq_tgt))
                if(cnt_p2 > cnt_min)
                    cnt_p2 = cnt_p2 - 1;
                    cnt_ps = cnt_p1 + cnt_p2;
                else
                    cnt_p1 = cnt_p1 + 1;
                    cnt_ps = cnt_p1 + cnt_p2;
                end
            end

            % Update cur_fw
            cur_fw = rw_update_cur_fw(ctrl.trq_tgt, frctn_int, cnt_p1, cnt_p2, cnt_ps, dat.spd_m, dat.temp, par);
    fprintf('1cur_fw %9.6f\n',cur_fw);
            
    % Check cur_fw
            while(abs(cur_fw) > par.cur_max)
                if(cnt_p1 > cnt_min)
                    cnt_p1 = cnt_p1 - 1;
                    cnt_ps = cnt_p1 + cnt_p2;
                else
                    cnt_p2 = cnt_p2 + 1;
                    cnt_ps = cnt_p1 + cnt_p2;
                end

                cur_fw = rw_update_cur_fw(ctrl.trq_tgt,frctn_int, cnt_p1, cnt_p2, cnt_ps, dat.spd_m, dat.temp, par);
                fprintf('2cur_fw %9.6f\n',cur_fw);
            end
            

            p1_ready = 0;
        else
%   fprintf('Neg Direct Control spd:%.3f\n',dat.spd_m/rps2rpm);
            ctrl.cur_cmd = cur_stgt;
            p1_ready = 1;
            
            cnt_p1 = cnt_min;
            cnt_p2 = cnt_min;
            cnt_ps = 2*cnt_min;

        end

    end

    % Update Count Set
    p_cnt_old = p_cnt_old + cnt_ps;

%  fprintf('new tot: %d ps: %d p1:%d  p2:%d\n',ctrl.open_trq.cnt_tot,cnt_ps, cnt_p1,cnt_p2);    
elseif ((p_cnt_old + cnt_p1) <= ctrl.open_trq.cnt_tot)
    fprintf('p1 ready\n');
    if (p1_ready == 0)
    
        fprintf('Set current\n');
        ctrl.cur_cmd = cur_fw;
        p1_ready = 1;
    end

end



end