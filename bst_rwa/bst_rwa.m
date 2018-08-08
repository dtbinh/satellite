close all
clear all
clc
format long

model_type = 'BST_RW_MODEL_TYPE_GENERAL_BLDC';
ctrl_mode  = 'BST_RW_CTRL_MODE_TORQUE';

% IADCS  SPEED CONTROL PROPAGATION MODEL PARAMETERS
model_k = 1.000000000000000;
model_d = 2.784230231948520;
model_t = 0.556846046389705;

% CONTROL MODE TORQUE
kp = 76; 
ki = 477; 
kd = 0.0;

% 
bldc_ke = 0.002115; % [V/rpm]
bldc_R  = 8.6;                % [Ohm]
bldc_n  = 1.00;               % [-] Efficiency
bldc_km = 0.0202;             % [Nm/A]

U_sup = 23.1;   %[V]


% Speed control
wtgt = 1000/60*2*pi; %[rad/s]
wmax = 5000;

% Torque Control
Ttgt = -0.00100;    %[Nm]

% Current control
rw_tgt = 0.5; % [A]
imax = 0.82; % [A]

     
I = 0.0009785480;

dt   = 0.05;
tdur = 120;
t    = 0:dt:tdur;

 % PID Control
     pid_kp = 20.0; 
     pid_ki = 0.2; 
     pid_kd = 0.0;
Ttgt = Ttgt*ones(tdur/dt+1,1);
wtgt = wtgt*ones(tdur/dt+1,1);

rw_w(1) = 0;      % [rpm] Initial Angular Velocity
rw_h(1) = I*rw_w(1);


for i = 1:1:tdur/dt+1
 if strcmp(model_type ,'BST_RW_MODEL_TYPE_GENERAL_BLDC')
%          
% 	T_err   = Ttgt(i) - T(i);
% 	T_err_i = T_err_i_old + T_err*dt;
% 	% itgt(i) = pid_kp*T_err  +  pid_ki*T_err_i  +  pid_kd*((T_err - T_err_old)/dt);
%      

    % Set Current Target
    rw_cur_cmd = rw_tgt;


    h = rw_h(i);
     
	% bst_rwa_dynamics_rwa05
    
    spd_rpm = dat->rw[i].sp*rps2rpm;
    
	if abs(rw_w(1)) > wmax
        cur(i) = 0;
    else

        cur(i)     = sat(itgt(i),imax); % [A] current saturation check

        % Adapt current due to Back-EMF
        cur_max_bemf = (U_sup - abs(bldc_ke*w(i))) / bldc_R; % 
        cur(i)       = sat(cur(i),cur_max_bemf);                 % Back-EMF limits the maximum current
        
    end
        
    trq = bldc_n*bldc_km*cur(i);
    
    % Torque friction
    trq_friction(i+1) = bst_rwa_friction_model(w(i));

    hdot = trq + trq_friction(i+1); 
 
    % Update
    rw_h(i+1)   = h + trq*dt;
    rw_trq(i+1) = trq;
    rw_sp(i+1)  = 1/I*rw_h(i+1);    
    rw_cur(i+1) = rw_cur_motor;
    
     
%      % Save
%      T_err_old   = T_err;
%      T_err_i_old = T_err_i;
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
plot(t,(w(1:end-1)),'-')
grid on
xlabel('Time [s]');
ylabel('Wheel Angular Velocity [rpm]'); 

subplot(2,2,2)
plot(t,T(1:end-1),'-')
grid on
xlabel('Time [s]');
ylabel('Torque [Nm]'); 

subplot(2,2,3)
plot(t,cur,'-')
grid on
xlabel('Time [s]');
ylabel('Current [A]');

subplot(2,2,4)
plot(t,h(1:end-1),'-')
grid on
xlabel('Time [s]');
ylabel('Friction [Nm]'); 