function output = filter_kalman_sequential(input)
w_B_BI_m      = input(1:3,1);  % Measured Angular Velocity (Gyro)

q_B_I_m(:,1)  = input(4:7,1);  % Measured Quaternion (Star Tracker 1)
q_B_I_m(:,2)  = input(8:11,1); % Measured Quaternion (star Tracker 2)

S_B_m         = input(12:14,1); % Measured Sun vector in Sensor Frame
B_B_m         = input(15:17,1);   % Measured Magnetic Field in Body Frame
S_I           = input(18:20,1);   % S Sun Vector in Inertial Frame since we have data of the Sun
B_I           = input(21:23,1);  % B Magnetic Vector in Inertial Frame since we have data of the Magnetic Field
T_m           = input(24:26,1);  % Measured Temperature, Referenced Temperaure, N/A
T_c           = input(27:29,1);  % Control Torque Desired
s_flag        = input(30:32,1);  % Sensor Flag [Gyro;ST1;ST2]


e_B_I_m  = q_B_I_m(1:3,:);% Measured Quaternion Euler Vector Only (Star Tracker 1 & 2)

%% PARAMETERS
global CONST
global FLTR

t = get_param(CONST.model,'SimulationTime');
if (FLTR.dbskf) 
        fprintf('\n\n------------------Time %.4f------------------',t);
end

dt      = CONST.dt;         % Sampling Time of Kalman Filter

sig_v   = CONST.sig_v;      % Noise Standard Deviation Attitude State
sig_u   = CONST.sig_u;      % Noise Standard Deviation Bias State
sig_w   = CONST.sig_w;          % Noise Standard Deviation Angular Velocity State

sig_st  = CONST.sig_st; 	% Star Tracker Standard Deviation (average)
sig_ss  = CONST.sig_ss;	    % Sun Sensor Standard Deviation
sig_mg  = CONST.sig_mg;     % Magnetometer Standard Deviation
mflag   = CONST.mflag;      % Sensors Flag (Star Tracker, SS1, SS2, Mag)
I       = CONST.I;          % Spacecraft Moments of Inertia


mflag(1) = s_flag(2)*mflag(1);  % ST1 Flag
mflag(2) = s_flag(3)*mflag(2);  % ST2 Flag
mflag(5) = s_flag(1)*mflag(5);  % Gyro Flag


%% LOOP VALUES

persistent qk biask wk Pk coefk wkm dwk;  % Variables that are retained in memory between calls to the function.

%% INITIAL VALUES
if isempty(qk)
    if (FLTR.dbskf) 
        fprintf('\nInitialisation');
    end
    
    % Initial Internal Loop
    qk    = FLTR.qk;                % Initial Quaternion (xyzw) for internal variable
    biask = FLTR.bk;                % Initial Bias Value from Lab Calibration
    coefk = zeros(3,1);             % Initial Temp Coeff
    wk    = FLTR.wk;                % Initial Value from measured gyro
    wkm   = FLTR.wk;                % Initial Value from measured gyro
    
    Pk    = FLTR.Pk_skf; % Initial Error Covariance 
    
    % Initial Output
    output(1:3,1)     = wk;
    output(4:6,1)     = biask;
    output(7:9,1)     = coefk;
    output(1:12,2:13) = Pk;
    output(1:4,14)    = qk; % Initial Quaternion (wxyz) for external output
    output(5:7,14)    = [0;0;0];   % Initial Euler Angles
    
    return;
end

MaxST  = 2;  % Index of last Star Tracker
MaxSS  = 3;  % Index of last Sun Sensor
MaxMag = 4;  % Index of last Magnetometer
MaxGg  = 5;  % Index of last Gyro

%% MEASUREMENT
if (FLTR.dbskf) 
    fprintf('\nMeasurements');
end


for i = 1:length(mflag)
    
if (mflag(i)==1)
    
        % Reset
        delX   = zeros(12,1); 
        
    if (FLTR.dbskf) 
        fprintf('\nMeasurements %d',i);
    end
 
    if( (1)&&(mflag(i) == 1) && (i <= MaxST) )
            % Star Tracker
            if (FLTR.dbskf) 
                fprintf('\nStar Tracker Measurements');
                
            end
            
              
            % Sensitivity
            Xi = q2xi(qk);                     
            H  = [1/2*Xi(1:3,:) zeros(3,3) zeros(3,3) zeros(3,3)];
            H_st  = [1/2*Xi(1:3,:) zeros(3,3) zeros(3,3) zeros(3,3)];
            R  = sig_st(i)^2*eye(3);             
            
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);        
            K_st = Pk*H'/(H*Pk*H' + R);
            
            % Update 
            res  = e_B_I_m(1:3,i) - qk(1:3,1); % quaternion
            delX = K*(res);       % delta angle 
            

	elseif( (mflag(i) == 1) && (i <= MaxSS) ) 
            % Sun Sensor
            if (FLTR.dbskf) 
                fprintf('\nSun Sensor Measurements');
            end
            
            % Sensitivity
            R_B_I  = q2xi(qk)'*q2psi(qk);
            H = [smtrx(R_B_I*S_I) zeros(3,3) zeros(3,3) zeros(3,3)];  
            
            R = sig_ss^2*eye(3);                            
            
            % Gain
            K   = Pk*H'/(H*Pk*H' + R);            
            
            % Update               
            res = S_B_m - R_B_I*S_I;  % Sensor Frame
            delX = K*(res); 

    elseif( (mflag(i) == 1) && (i <= MaxMag) ) 
            % Magnetometer
            if (FLTR.dbskf) 
                fprintf('\nMagnetometer Measurements');
            end
            
            % Sensitivity
            R_B_I  = q2xi(qk)'*q2psi(qk); 
            H = [smtrx(R_B_I*B_I) zeros(3,3) zeros(3,3) zeros(3,3)];
            R = sig_mg^2*eye(3);
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);    
            
            % Update
            res  = B_B_m - R_B_I*B_I;       
            delX = K*(res);

     end
     
%% UPDATE

if (FLTR.dbskf) 
    fprintf('\nUpdate');
end

% Update of Quaternion(xyzw)
qk    = qk  +  1/2*q2xi(qk)*delX(1:3,:);  
qk    = qnorm(qk); 

% Update of Bias  
biask = biask  +  delX(4:6,:);

% Update of Temperature Coefficient
coefk = coefk  +  delX(7:9,:);          

% Update of Angular Velocity
dwk   = delX(10:12,:);
wk    = wk  + dwk;

end
     
end

Pk  = (eye(12) - K_st*H_st)*Pk;  

% Update of Gyro Measurement
wkm   = w_B_BI_m  -  biask ; %  Estimated Angular Velocity with Noise


%% ERROR COVARIANCE PROPAGATION
% State Transition Matrix
F  = [-smtrx(wkm)   -eye(3)   zeros(3)    zeros(3);
       zeros(3)    zeros(3)   zeros(3)    zeros(3);
       zeros(3)    zeros(3)   zeros(3)    zeros(3);
       zeros(3)    zeros(3)   zeros(3)    zeros(3)];


Phi = eye(12) + F*dt + 0.5*(F*dt)^2;

% Discrete Process Noise Covariance
Qk = [(sig_v^2*dt+1/3*(sig_u^2)*dt^3)*eye(3)  -(1/2*sig_u^2*dt^2)*eye(3)           zeros(3)            zeros(3);
            -(1/2*sig_u^2*dt^2)*eye(3)             (sig_u^2*dt)*eye(3)             zeros(3)            zeros(3);
                   zeros(3)                             zeros(3)                   zeros(3)            zeros(3);
                   zeros(3)                             zeros(3)                   zeros(3)            zeros(3)];


Pk = Phi*Pk*Phi'+ Qk;           

%% STATE PROPAGATION
if (FLTR.dbskf) 
    fprintf('\nPropagate');
end

% Quaternion State 
psik  = sin(1/2*norm(wkm)*dt)/norm(wkm)*wkm;
omega = [cos(1/2*norm(wkm)*dt)*eye(3)-smtrx(psik)       psik;
                -psik'                       cos(1/2*norm(wkm)*dt) ];


qk   = omega*qk;      

% Angular Velocity State
dFdw   = I^-1*(-smtrx(wk)*I  +  smtrx(I*wk)); 
phi_dw = eye(3) + dFdw*dt  +  0.5*(dFdw*dt)^2;

B    = I^-1*eye(3);
Gmm  = B*dt  +  0.5*B*dFdw*dt^2;  

wk  = wkm; 


%% OUTPUT
w_B_BI_f = wk;                          % Angular Velocity of Body wrt to Inertial Frame 
q_B_I_f  = qk;                          % Quaternion (xyzw) from Inertia to Body
R_B_I_f  = q2dcm(q_B_I_f,'xyzw','tsf'); % Transformation Matrix from Inertia to Body
e_B_I_f  = dcm2eul(R_B_I_f,'zyx');      % Euler Angles for Transformation from Inertia to Body

output(1:3,1)     = w_B_BI_f;
output(4:6,1)     = biask;
output(7:9,1)     = coefk;
output(1:12,2:13) = Pk;
output(1:4,14)    = q_B_I_f;
output(5:7,14)    = e_B_I_f;
end

