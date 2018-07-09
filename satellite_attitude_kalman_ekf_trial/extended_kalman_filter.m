function output = extended_kalman_filter(input)
%% INPUT
w_B_BI_m = input(1:3,1);   % Measured Angular Velocity (Gyro)
e_B_I_m  = input(1:3,2:3); % Measured Quaternion Euler Vector Only (Star Tracker 1 & 2)
S_S_m    = input(1:3,4:7); % Measured Sun vector in Sensor Frame
B_B_m    = input(1:3,8);   % Measured Magnetic Field in Body Frame
S_I      = input(1:3,9);   % S Sun Vector in Inertial Frame since we have data of the Sun
B_I      = input(1:3,10);  % B Magnetic Vector in Inertial Frame since we have data of the Magnetic Field
T_m      = input(1:3,11);  % Measured Temperature, Referenced Temperaure, N/A
T_c      = input(1:3,12);  % Control Torque Desired
s_flag   = input(1:3,13);  % Sensor Flag [Gyro;ST1;ST2]

global CONST
dt      = CONST.dt;         % Sampling Time of Kalman Filter

sig_v   = CONST.sig_v;      % Noise Standard Deviation Attitude State
sig_u   = CONST.sig_u;      % Noise Standard Deviation Bias State
sig_w   = 1e-5   ;          % Noise Standard Deviation Angular Velocity State

sig_st  = CONST.sig_st; 	% Star Tracker Standard Deviation (average)
sig_ss  = CONST.sig_ss;	    % Sun Sensor Standard Deviation
sig_mg  = CONST.sig_mg;     % Magnetometer Standard Deviation
mflag   = CONST.mflag;      % Sensors Flag (Star Tracker, SS1, SS2, Mag)
I       = CONST.I;          % Spacecraft Moments of Inertia

dTk     = T_m(1)-T_m(2);    % Temperature Difference
uk      = T_c;              % Torque Input/ Control Torque

mflag(1) = s_flag(2)*mflag(1);  % ST1 Flag
mflag(2) = s_flag(3)*mflag(2);  % ST2 Flag
mflag(8) = s_flag(1)*mflag(8);  % Gyro Flag
%% LOOP VALUES

persistent qk biask wk Pk coefk wkm dwk;  % Variables that are retained in memory between calls to the function.

%% INITIAL VALUES
if isempty(qk)
    
    % Initial Internal Loop
    qk    = [0;0;0;1];                           % Initial Quaternion (xyzw) for internal variable
    biask = [0.0;0.0;0.0]*pi/180;                % Initial Bias Value from Lab Calibration
    coefk = zeros(3,1);                          % Initial Temp Coeff
    wk    = [0;0;0];                            % Initial Value from measured gyro
    wkm   = [0;0;0];                            % Initial Value from measured gyro
    dwk   = [0;0;0];
    
    sig_n = sqrt(sig_st(1)^2+sig_st(2)^2+sig_ss^2+sig_mg^2);                     % Initial Scalar sig_n
    Pk    = dt^(1/4)*sig_n^(1/2)*(sig_v^2+2*sig_u*sig_v*dt^(1/2))^(1/4)*eye(12); % Initial Error Covariance - does not matter much

    % Initial Output
    output(1:3,1)     = wk;
    output(4:6,1)     = biask;
    output(7:9,1)     = coefk;
    output(1:12,2:13) = Pk;
    output(1:4,14)    = [1;0;0;0]; % Initial Quaternion (wxyz) for external output
    output(5:7,14)    = [0;0;0];   % Initial Euler Angles
    
    return;
end

MaxST  = 2;  % Index of last Star Tracker
MaxSS  = 6;  % Index of last Sun Sensor
MaxMag = 7;  % Index of last Magnetometer
MaxGg  = 8;  % Index of last Gyro
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles

type = 1;


%% MEASUREMENT

R_B_I  = ATT(qk);    % Attitude Transformation Matrix of  Quaternion (xyzw) from Inertial to Body
delX   = zeros(12,1); % Matrix Initiation for delX

for i = 1:length(mflag)
     if( (mflag(i) == 1) && (i <= MaxST) )
            % Star Tracker Measurement 
            Xi = XI(qk);                     
            H  = [1/2*Xi(1:3,:) zeros(3,3) zeros(3,3) zeros(3,3) ]; 
            R  = sig_st(i)^2*eye(3);             
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);        
            
            % Update
            Pk   = (eye(12) - K*H)*Pk;          
            res  = e_B_I_m(1:3,i) - qk(1:3,1); % quaternion
            delX = delX + K*(res-H*delX);       % delta angle 
         

     elseif( (mflag(i) == 1) && (i <= MaxSS) ) 
            % Sun Sensor Measurement
            R_S_B = dcm(SSaxis(i-MaxST),SSangles(i-MaxST)); 
            H = [R_S_B*smtrx(R_B_I*S_I) zeros(3,3) zeros(3,3) zeros(3,3)];         
            R = sig_ss^2*eye(3);                            
            
            % Gain
            K   = Pk*H'/(H*Pk*H' + R);            
            
            % Update
            Pk  = (eye(12) - K*H)*Pk;                 
            res = S_S_m(:,i-MaxST) - R_S_B*R_B_I*S_I; 
            delX = delX + K*(res-H*delX);             

     elseif( (mflag(i) == 1) && (i <= MaxMag) ) 
            % Magnetometer
            H = [smtrx(R_B_I*B_I) zeros(3,3) zeros(3,3) zeros(3,3)];
            R = sig_mg^2*eye(3);
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);    
            
            % Update
            Pk   = (eye(12) - K*H)*Pk;      
            res  = B_B_m - R_B_I*B_I;       
            delX = delX + K*(res-H*delX);
            
    elseif( (mflag(i) == 1) && (i <= MaxGg) ) 
            % Gyroscope Measurements
            if (type == 1)
                H = [zeros(3,3) zeros(3,3) zeros(3,3) eye(3)];
            elseif (type ==2)
                H = [zeros(3,3) eye(3) zeros(3,3) eye(3)];
            end
            R = sig_v^2*eye(3);
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);    
            
            % Update
            Pk  = (eye(12) - K*H)*Pk;      
            res  =  w_B_BI_m  - wk - biask;       
            delX = delX + K*(res-H*delX); 
            
    end
end

%% UPDATE
% Update of Quaternion(xyzw)
qk    = qk+1/2*XI(qk)*delX(1:3,:);   
qk    = qnorm(qk);               

% Update of Bias  
biask = biask + delX(4:6,:);          

% Update of Temperature Coefficient
coefk = coefk + delX(7:9,:);          

% Update of Angular Velocity
dwk   = delX(10:12,:);
wk    = wk + dwk;

% Update of Gyro Measurement
wkm   = w_B_BI_m - biask ; %  Estimated Angular Velocity with Noise

%% ERROR COVARIANCE PROPAGATION
if (type == 1)
    F  = [-smtrx(wkm)   -eye(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3)];
elseif (type ==2)
    F  = [-smtrx(wk)   zeros(3)   zeros(3)      eye(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    I^-1*(-smtrx(wk)*I+smtrx(I*wk)-smtrx(dwk)*I+smtrx(I*dwk))];
end  
Phi = eye(12) + F*dt + 0.5*(F*dt)^2;

% Discrete Process Noise Covariance
if (type == 1)
    Qk = [(sig_v^2*dt+1/3*(sig_u^2)*dt^3)*eye(3)  -(1/2*sig_u^2*dt^2)*eye(3)           zeros(3)            zeros(3);
                -(1/2*sig_u^2*dt^2)*eye(3)             (sig_u^2*dt)*eye(3)             zeros(3)            zeros(3);
                       zeros(3)                             zeros(3)                   zeros(3)            zeros(3);
                       zeros(3)                             zeros(3)                   zeros(3)       (sig_w^2*dt)*eye(3)];

elseif (type ==2)

    Qk = [ (sig_v^2*dt+1/3*sig_w^2*dt^3)*eye(3)           zeros(3)                   zeros(3)       (1/2*sig_w^2*dt^2)*eye(3);
                     zeros(3)                        (sig_u^2*dt)*eye(3)             zeros(3)            zeros(3);
                     zeros(3)                             zeros(3)                   zeros(3)            zeros(3);
                (1/2*sig_w^2*dt^2)*eye(3)                 zeros(3)                   zeros(3)          (sig_w^2*dt)*eye(3)];
end
Pk = Phi*Pk*Phi'+ Qk;       

%% STATE PROPAGATION 
% Quaternion State 
if (type ==1)
    psik  = sin(1/2*norm(wkm)*dt)/norm(wkm)*wkm;
    omega = [cos(1/2*norm(wkm)*dt)*eye(3)-smtrx(psik)       psik;
                    -psik'                       cos(1/2*norm(wkm)*dt) ]; 
elseif (type ==2)
    psik  = sin(1/2*norm(wk)*dt)/norm(wk)*wk;
    omega = [cos(1/2*norm(wk)*dt)*eye(3)-smtrx(psik)       psik;
                    -psik'                       cos(1/2*norm(wk)*dt) ];
end

qk   = omega*qk;      

% Angular Velocity State
dFdw   = I^-1*(-smtrx(wk)*I+smtrx(I*wk));
phi_dw = eye(3) + dFdw*dt + 0.5*(dFdw*dt)^2;

B      = I^-1*eye(3);
Gmm    = B*dt+ 0.5*B*dFdw*dt^2;  

wk    = phi_dw*wk + Gmm*uk;                                

%% OUTPUT
w_B_BI_f = wk;
q_B_I_f  = qinvert(qk,'xyzw');         % Quaternions [w;x;y;z] 
R_B_I_f  = q2dcm(q_B_I_f,'wxyz','tsf'); % Transformation Matrix from Inertia to Body
e_B_I_f  = dcm2eul(R_B_I_f,'zyx');      % Euler Angles for Transformation from Inertia to Body

output(1:3,1)     = w_B_BI_f;
output(4:6,1)     = biask;
output(7:9,1)     = coefk;
output(1:12,2:13) = Pk;
output(1:4,14)    = q_B_I_f;
output(5:7,14)    = e_B_I_f;
end

