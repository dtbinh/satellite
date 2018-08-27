function output = extended_kalman_filter(input)
w_B_BI_m = input(1:3,1);   % Measured Angular Velocity (Gyro)

q_B_I_m(:,1)  = input(4:7,1);
q_B_I_m(:,2)  = input(8:11,1);

S_S_m(:,1)    = input(12:14,1); % Measured Sun vector in Sensor Frame
S_S_m(:,2)    = input(15:17,1); % Measured Sun vector in Sensor Frame
S_S_m(:,3)    = input(18:20,1); % Measured Sun vector in Sensor Frame
S_S_m(:,4)    = input(21:23,1); % Measured Sun vector in Sensor Frame

B_B_m    = input(24:26,1);   % Measured Magnetic Field in Body Frame
S_I      = input(27:29,1);   % S Sun Vector in Inertial Frame since we have data of the Sun
B_I      = input(30:32,1);  % B Magnetic Vector in Inertial Frame since we have data of the Magnetic Field
T_m      = input(33:35,1);  % Measured Temperature, Referenced Temperaure, N/A
T_c      = input(36:38,1);  % Control Torque Desired
s_flag   = input(39:41,1);  % Sensor Flag [Gyro;ST1;ST2]

e_B_I_m  = q_B_I_m(1:3,:);% Measured Quaternion Euler Vector Only (Star Tracker 1 & 2)


global CONST
global FLTR

t = get_param(CONST.model,'SimulationTime');
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
    qk    = qnorm([0.0;0.0;0.0;1]);                           % Initial Quaternion (xyzw) for internal variable
    biask = [0.0;0.0;0.0]*pi/180;                % Initial Bias Value from Lab Calibration
    coefk = zeros(3,1);                          % Initial Temp Coeff
    wk    = [0;0;0];                            % Initial Value from measured gyro
    wkm   = [0;0;0];                            % Initial Value from measured gyro
    dwk   = [0;0;0];
    
    sig_n = sqrt(sig_st(1)^2  +  sig_st(2)^2  +  sig_ss^2  + sig_mg^2);                     % Initial Scalar sig_n
    Pk    = dt^(1/4)*sig_n^(1/2)*(sig_v^2  +  2*sig_u*sig_v*dt^(1/2))^(1/4)*eye(12); % Initial Error Covariance - does not matter much
    Pk    = [(1)^2*eye(3)      zeros(3)  zeros(3)  zeros(3); 
                zeros(3)   (3*pi/180)^2*eye(3)  zeros(3)  zeros(3); 
                zeros(3)       zeros(3)  zeros(3)  zeros(3);
                zeros(3)       zeros(3)  zeros(3)  zeros(3)]; % Initial Error Covariance 
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
MaxSS  = 6;  % Index of last Sun Sensor
MaxMag = 7;  % Index of last Magnetometer
MaxGg  = 8;  % Index of last Gyro
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles

for i =1:length(S_S_m);
R_S_B = dcm(SSaxis(i),SSangles(i));
S_B_m(:,i)    = R_S_B'*S_S_m(:,i);
end


%% MEASUREMENT

R_B_I  = q2xi(qk)'*q2psi(qk);    % Attitude Transformation Matrix of  Quaternion (xyzw) from Inertial to Body
delX   = zeros(12,1); % Matrix Initiation for delX

for i = 1:length(mflag)
     if( (1)&&(mflag(i) == 1) && (i <= MaxST) )
            % Star Tracker Measurement 
            Xi = q2xi(qk);                     
            H  = [1/2*Xi(1:3,:) zeros(3,3) zeros(3,3) zeros(3,3)]; 
            R  = sig_st(i)^2*eye(3);             
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);        
            
            % Update
            Pk   = (eye(12) - K*H)*Pk;          
            res  = e_B_I_m(1:3,i) - qk(1:3,1); % quaternion
            delX = delX + K*(res-H*delX);       % delta angle 
            
     elseif ( (0)&&(mflag(i) == 1) && (i <= MaxST) )
         % BST

            % Star Tracker Measurement 
            Xi = q2xi(qk);                     
            H  = [eye(3,3) zeros(3,3) zeros(3,3) zeros(3,3) ]; 
            R  = sig_st(i)^2*eye(3);             
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);        
            
            % Update
            Pk   = (eye(12) - K*H)*Pk;           
            res  = 2*Xi'*q_B_I_m(:,i);        % [3x1]
            delX = delX + K*(res-H*delX);       % delta angle 

     elseif( (mflag(i) == 1) && (i <= MaxSS) ) 
            % Sun Sensor Measurement          
            H = [smtrx(R_B_I*S_I) zeros(3,3) zeros(3,3) zeros(3,3)];  
            
            R = sig_ss^2*eye(3);                            
            
            % Gain
            K   = Pk*H'/(H*Pk*H' + R);            
            
            % Update
            Pk  = (eye(12) - K*H)*Pk;                 
            res = S_B_m(:,i-MaxST) - R_B_I*S_I;  % Sensor Frame
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
            
    elseif( (mflag(i) == 1) && (i <= MaxGg)) 
        
            % Gyroscope Measurements
            switch FLTR.mode
                case 0
                    H = [ zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)];
                case 1
                    H = [ zeros(3,3) zeros(3,3) zeros(3,3)   eye(3)  ];
                case 2
                    H = [ zeros(3,3)   eye(3)    zeros(3,3)   eye(3)  ];
            end
            
            R = sig_v^2*eye(3);
            
            % Gain
            K = Pk*H'/(H*Pk*H' + R);    
            
            % Update
            Pk  = (eye(12) - K*H)*Pk;      
            res  =  (w_B_BI_m - biask)  - wk ;       
            delX = delX + K*(res - H*delX); 
            
    end
end



%% UPDATE
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

% Update of Gyro Measurement

if (mflag(8)) 
    wkm   = w_B_BI_m  -  biask ; %  Estimated Angular Velocity with Noise
end

%% ERROR COVARIANCE PROPAGATION
switch FLTR.mode
    case 0
    F  = [-smtrx(wkm)   -eye(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3)];
    case 1
    F  = [-smtrx(wkm)   -eye(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3)];
    case 2
    F  = [-smtrx(wk)   zeros(3)   zeros(3)     eye(3) ;
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    zeros(3);
           zeros(3)    zeros(3)   zeros(3)    I^-1*(-smtrx(wk)*I+smtrx(I*wk)-smtrx(dwk)*I+smtrx(I*dwk))];
end  

Phi = eye(12) + F*dt + 0.5*(F*dt)^2;

% Discrete Process Noise Covariance
switch FLTR.mode
    case 0
    Qk = [(sig_v^2*dt+1/3*(sig_u^2)*dt^3)*eye(3)  -(1/2*sig_u^2*dt^2)*eye(3)           zeros(3)            zeros(3);
                -(1/2*sig_u^2*dt^2)*eye(3)             (sig_u^2*dt)*eye(3)             zeros(3)            zeros(3);
                       zeros(3)                             zeros(3)                   zeros(3)            zeros(3);
                       zeros(3)                             zeros(3)                   zeros(3)            zeros(3)];

    case 1
    Qk = [(sig_v^2*dt+1/3*(sig_u^2)*dt^3)*eye(3)  -(1/2*sig_u^2*dt^2)*eye(3)           zeros(3)            zeros(3);
                -(1/2*sig_u^2*dt^2)*eye(3)             (sig_u^2*dt)*eye(3)             zeros(3)            zeros(3);
                       zeros(3)                             zeros(3)                   zeros(3)            zeros(3);
                       zeros(3)                             zeros(3)                   zeros(3)       (sig_w^2*dt)*eye(3)];

    case 2
    Qk = [ (sig_v^2*dt+1/3*sig_w^2*dt^3)*eye(3)           zeros(3)                   zeros(3)       (1/2*sig_w^2*dt^2)*eye(3);
                     zeros(3)                        (sig_u^2*dt)*eye(3)             zeros(3)            zeros(3);
                     zeros(3)                             zeros(3)                   zeros(3)            zeros(3);
                (1/2*sig_w^2*dt^2)*eye(3)                 zeros(3)                   zeros(3)          (sig_w^2*dt)*eye(3)];
end

Pk = Phi*Pk*Phi'+ Qk;           

%% STATE PROPAGATION 
% Quaternion State 
switch FLTR.mode
    case 0
    psik  = sin(1/2*norm(wkm)*dt)/norm(wkm)*wkm;
    omega = [cos(1/2*norm(wkm)*dt)*eye(3)-smtrx(psik)       psik;
                    -psik'                       cos(1/2*norm(wkm)*dt) ];
    case 1
    psik  = sin(1/2*norm(wkm)*dt)/norm(wkm)*wkm;
    omega = [cos(1/2*norm(wkm)*dt)*eye(3)-smtrx(psik)       psik;
                    -psik'                       cos(1/2*norm(wkm)*dt) ]; 
    case 2
    psik  = sin(1/2*norm(wk)*dt)/norm(wk)*wk;
    omega = [cos(1/2*norm(wk)*dt)*eye(3)-smtrx(psik)       psik;
                    -psik'                       cos(1/2*norm(wk)*dt) ];
end

qk   = omega*qk;      

% Angular Velocity State
dFdw   = I^-1*(-smtrx(wk)*I  +  smtrx(I*wk)); 
phi_dw = eye(3) + dFdw*dt  +  0.5*(dFdw*dt)^2;

B      = I^-1*eye(3);
Gmm    = B*dt  +  0.5*B*dFdw*dt^2;  

switch FLTR.mode
    case 0
        wk  = wkm; 
    case 1
        wk  = phi_dw*wk  +  Gmm*uk;
    case 2
        wk  = phi_dw*wk  +  Gmm*uk;  
end


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

