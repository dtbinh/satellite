close all
clear all
clc
format long

model_type = 'BST_RW_MODEL_TYPE_GENERAL_BLDC';
ctrl_mode  = 'BST_RW_CTRL_MODE_CURRENT';


% RWA PARAMETERS     
rw_moi      = 0.0009785480;
rw_bldc_ke  = 0.002115;        % [V/rpm]
rw_bldc_R   = 8.6;             % [Ohm]
rw_bldc_n   = 1.00;            % [-] Efficiency
rw_bldc_km  = 0.0202;          % [Nm/A]

rw_U_sup = 23.1;   %[V]

% IADCS  SPEED CONTROL PROPAGATION MODEL PARAMETERS
model_k = 1.000000000000000;
model_d = 2.784230231948520;
model_t = 0.556846046389705;

% CONTROL MODE TORQUE
kp = 76; 
ki = 477; 
kd = 0.0;

% Speed control
rw_n_max = 5000*rpm2rps;      %[rps]

% Torque Control
rw_T_tgt = 0.0040;    %[Nm]

% Current control
rw_I_tgt = 0.00; % [A]
rw_I_max = 0.82; % [A]


dt   = 0.01;        % [s]
tdur = 600;
t    = 0:dt:tdur;


% Intial Condition
rw_sp(1)  = -5000*rpm2rps;       % [rps] Initial Angular Velocity
rw_h(1)   = rw_moi*rw_sp(1);
rw_trq(1) = 0;   
rw_cur(1) = 0;
rw_cur_motor = 0;

trq_err_i     = 0;
trq_err_old   = 0;

% PID Control
global pid

pid.kp = 20.0; 
pid.ki = 15.00; 
pid.kd = 0.00;

pid.init   = 1;
pid.aw_thr = rw_I_max;
pid.aw_flg = 0;
pid.aw_fact = 0.7;
pid.intvl  = 0.1;      % [s]
is_int_cur = 1 ;

for i = 1:1:tdur/dt+1


if strcmp(model_type ,'BST_RW_MODEL_TYPE_GENERAL_BLDC')
  
switch ctrl_mode
    case 'BST_RW_CTRL_MODE_CURRENT'
            rw_cur_cmd = rw_I_tgt;
            
    case 'BST_RW_CTRL_MODE_TORQUE'
        
        if (pid.init ==1)
                rw_cur_cmd = 1.0/rw_bldc_km * (rw_T_tgt - bst_rwa_friction_model(rw_sp(i)*rps2rpm));
        end
        
        
        if (rem(t(i),pid.intvl) == 0)
            rw_cur_cmd = bst_pid_update(rw_cur_cmd,rw_T_tgt - rw_trq(i), pid, pid.intvl);
            pid.init = 0;
        end
        
        
    case 'BST_RW_CTRL_MODE_SPEED'
        
        
end

     
%% EULER PROPAGATION
    % update the angular momentum
    h = rw_h(i);

        spd_rpm = rw_sp(i)*rps2rpm;  % [rpm] 
    
    % Speed Saturation check
	if abs(rw_sp(i)) > rw_n_max
        cur = 0.0;
    else
        cur = sat(rw_cur_cmd,rw_I_max); % [A] current saturation check

        % Adapt current due to Back-EMF
        cur_max_bemf = (rw_U_sup - abs(spd_rpm)*rw_bldc_ke) / rw_bldc_R; % 
        cur          = sat(cur,cur_max_bemf);                 % Back-EMF limits the maximum current
        
        % Internal Current Effect
        if (is_int_cur) && (rw_sp(i)*rw_cur_motor < 0)
            cur_bemf = spd_rpm*rw_bldc_ke / rw_bldc_R;
            cur      = cur - cur_bemf; 
        end
        
    end
    
    rw_cur_motor = cur;                % [A] current supplied to motor
    trq = cur*rw_bldc_n*rw_bldc_km;    
    
    % Torque friction
    trq_friction = bst_rwa_friction_model(spd_rpm);
    hdot         = trq + trq_friction; 
 
    %% Update the next step
    rw_h(i+1)   = h + hdot*dt;
    rw_trq(i+1) = hdot;
    rw_sp(i+1)  = 1/rw_moi*rw_h(i+1);    % [rps]
    rw_cur(i+1) = rw_cur_motor;
  
 end
 
%  if strcmp(model_type ,'BST_RW_MODEL_TYPE_HT_RW200_15')
%     
%      I = 1.546500e-06;
% 
%      if strcmp(ctrl_mode ,'BST_RW_CTRL_MODE_TORQUE')
%          if init == 1
%              fprintf('BST_RW_RW200_15_CTRL_MODE_TORQUE\n');
% 
%              init = 0;
%          end
%          
%          
%          wdot(i) = Ttgt(i)/I;
%          hdot(i) = I*wdot(i);
%          T(i) = hdot(i);
%          h(i) = h(i-1)+hdot(i)*dt;
%          w(i) = w(i-1)+wdot(i)*dt;
%          
%      elseif strcmp(ctrl_mode ,'BST_RW_CTRL_MODE_SPEED')
%          if init == 1
%          fprintf('BST_RW_RW200_15_CTRL_MODE_SPEED\n');
%          init = 0;
%          end
%          
%          Tdot(i) = I*(-2*model_d/(I*model_t)*hdot(i-1) + 1/(model_t^2)*(model_k*wtgt(i)-w(i-1)));   
%          dh = Tdot(i)*dt;
%          T(i) = T(i-1)+dh;
%          h(i) = h(i-1)+T(i)*dt;
%          w(i) = 1/I*h(i);
%          hdot(i)= T(i);
%      end
%  end
 
end


figure
subplot(2,2,1)
plot(t,(rw_sp(1:end-1)*rps2rpm),'-')
grid on
xlabel('Time [s]');
ylabel('Wheel Angular Velocity [rpm]'); 

subplot(2,2,2)
plot(t,rw_trq(1:end-1)*1e3,'-')
grid on
xlabel('Time [s]');
ylabel('Torque [mNm]'); 

subplot(2,2,3)
plot(t,rw_cur(1:end-1),'-')
grid on
xlabel('Time [s]');
ylabel('Current [A]');

subplot(2,2,4)
plot(t,rw_h(1:end-1),'-')
grid on
xlabel('Time [s]');
ylabel('Friction [Nm]'); 