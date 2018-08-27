function output = extended_kalman_filter(input)
w_B_BI_m  = input(1:3,1);   % wk1t Measured Angular Velocity (Gyro)
e_B_I_m   = input(1:3,2);   % yk1 Measured Quaternion Euler Vector Only (Star Tracker)
S_S_m     = input(1:3,3:6); % bs Measured Sun vector in Sensor Frame
B_B_m     = input(1:3,7);   % Bk1 
S_I       = input(1:3,8);   % S Sun Vector in Inertial Frame since we have data of the Sun
B_I       = input(1:3,9);   % B Magnetic Vector in Inertial Frame since we have data of the Magnetic Field
global CONST

dt      = CONST.dt;         % Sampling Time
sig_v   = CONST.sig_v;      % Noise Standard Deviation (gyro noise)
sig_u   = CONST.sig_u;      % Noise Standard Deviation (gyro bias)
sig_st  = CONST.sig_st; 	% Star Tracker Standard Deviation
sig_ss  = CONST.sig_ss;	    % Sun Sensor Standard Deviation
sig_mg  = CONST.sig_mg;    % Magnetometer Standard Deviation
mflag   = CONST.mflag;      % Sensors Flag (Star Tracker, SS1, SS2, Mag
%% LOOP VALUES

persistent qk biask wk Pk;  % Variables that are retained in memory between calls to the function.

%% INITIAL VALUES
if isempty(qk)
    
    % Initial Internal Loop
    qk    = [0;0;0;1];                           % Initial Quaternion (xyzw) for internal variable
    biask = zeros(3,1);                          % Initial Bias Value (xyz)
    wk    = w_B_BI_m;                            % Initial Value from measured gyro
    sig_n = sqrt(sig_st^2+sig_ss^2+sig_mg^2);    % Initial Scalar sig_n
    Pk    = dt^(1/4)*sig_n^(1/2)*(sig_v^2+2*sig_u*sig_v*dt^(1/2))^(1/4)*eye(6); % Initial Error Covariance - does not matter much

    % Initial Output
    output(1:3,1)    = wk;
    output(4:6,1)    = biask;
    output(1:6,2:7)  = Pk;
    output(1:4,8)    = [1;0;0;0]; % Initial Quaternion (wxyz) for internal variable
    output(1:3,9)    = [0;0;0];   % Initial Euler Angles
   
    return;
end

MaxST  = 1;  % Index of last Star Tracker
MaxSS  = 5;  % Index of last Sun Sensor
MaxMag = 6;  % Index of last Magnetometer
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles


%% DISCRETE PROPAGATION 
% Quaternion  
psik  = sin(1/2*norm(wk)*dt)/norm(wk)*wk;                              % [John_Crassidis] Equation 7.41
omega = [cos(1/2*norm(wk)*dt)*eye(3)-Smtrx(psik)       psik;
                 -psik'                       cos(1/2*norm(wk)*dt) ];  % [John_Crassidis] Equation 7.40
qk1   = omega*qk;                                                      % [John_Crassidis] Equation 7.39

% Bias 
biask1 = biask;                                                        % ?^k1- =  ?^k+ [John_Crassidis] Equation 7.42b                                               

% Error Covariance 
Phi_11 = eye(3)-Smtrx(wk)*sin(norm(wk)*dt)/norm(wk) + (Smtrx(wk))^2*(1-cos(norm(wk)*dt))/(norm(wk))^2;                      % [John_Crassidis] Equation 7.45
Phi_12 = Smtrx(wk)*(1-cos(norm(wk)*dt))/(norm(wk))^2 - eye(3)*dt -Smtrx(wk)^2*(norm(wk)*dt-sin(norm(wk)*dt))/norm(wk)^3;  % [John_Crassidis] Equation 7.45
Phi_21 = zeros(3);                                                                                                      % [John_Crassidis] Equation 7.45
Phi_22 = eye(3);                                                                                                        % [John_Crassidis] Equation 7.45
Phi    = [Phi_11 Phi_12; 
          Phi_21 Phi_22]; % Discrete error-state transition matrix

Gk = [ -eye(3) zeros(3); 
       zeros(3)  eye(3)];
  
Qk = [ (sig_v^2*dt+1/3*sig_u^2*dt^3)*eye(3) -(1/2*sig_u^2*dt^2)*eye(3);
         -(1/2*sig_u^2*dt^2)*eye(3)            (sig_u^2*dt)*eye(3)    ];    % Discrete Process Noise Covariance

    
Pk1 = Phi*Pk*Phi'+Gk*Qk*Gk';                                              % [John_Crassidis] Equation 7.43


%% GAIN & UPDATE (MURRELL'S VISION)
if(sum(mflag) >= 1)

    R_B_I  = ATT(qk1);   % A(q^k-)Attitude Transformation Matrix of  Quaternion (xyzw) from Inertial to Body

    delX   = zeros(6,1); % Matrix Initiation for delX
  
    
    for i = 1:MaxMag
        if( (mflag(i) == 1) && (i <= MaxST) )
%             Star Tracker Measurement Matrix (when i = 1) quaternion
            Xi = XI(qk1);                    % 
            H  = [1/2*Xi(1:3,:) zeros(3,3) ]; % Sensor Sensitivity Matrix
            R  = sig_st^2*eye(3);             % Sensor Noise
            
            % Gain
            K = Pk1*H'/(H*Pk1*H' + R);        % Kk = (Pk-)*(Hk)'/[(Hk)*(Pk-)*(Hk)'+R]
            
            % Update
            Pk1  = (eye(6) - K*H)*Pk1;          % Pk+ = [I - Kk*Hk]*(Pk-)
            res  = e_B_I_m(1:3,1) - qk1(1:3,1); % Residual y~k - q^k-
            delX = delX + K*(res-H*delX);       % ?xk^~ = Kk*[y~k-hk]
            
         elseif( (mflag(i) == 1) && (i <= MaxSS) ) 
            % Sun Sensor (when i = 2,3,4,5) Sun Body Vector
            R_S_B = DCM(SSaxis(i-MaxST),SSangles(i-MaxST)); % [3 x 3] Rotation Matrix from Body to Sensor Frame
            H = [R_S_B*Smtrx(R_B_I*S_I) zeros(3,3) ];         % [3 x 6] Sensitivity Matrix
            R = sig_ss^2*eye(3);                            % [3 x 3] Matrix
            
            % Gain
            K = (Pk1*H')/(H*Pk1*H' + R);            % Kk = (Pk-)*(Hk)'/[(Hk)*(Pk-)*(Hk)'+R]
            
            % Update
            Pk1 = (eye(6) - K*H)*Pk1;                 % Pk+ = [I - Kk*Hk]*(Pk-)
            res = S_S_m(:,i-MaxST) - R_S_B*R_B_I*S_I; % measured Sun angle (sensor) - sun angle (sensor)
            delX = delX + K*(res-H*delX);             % ?xk^~ = Kk*[y~k-hk]
            
         elseif( (mflag(i) == 1) && (i <= MaxMag) ) 
            % Magnetometer
            H = [SKEW(R_B_I*B_I) zeros(3,3)];
            R = sig_mg^2*eye(3);
            
            % Gain
            K = (Pk1*H')/(H*Pk1*H' + R);    % Kk = (Pk-)*(Hk)'/[(Hk)*(Pk-)*(Hk)'+R]
            
            % Update
            Pk1  = (eye(6) - K*H)*Pk1;      % Pk+ = [I - Kk*Hk]*(Pk-)
            res  = B_B_m - R_B_I*B_I;           % measured Magnetic Field (body) - Magnetic Field (body)
            delX = delX + K*(res-H*delX);   % dxk^~ = Kk*[y~k-hk]
            
        end
    end
    
%% UPDATE

    qk1    = qk1+1/2*XI(qk1)*delX(1:3,:); % Update of Quaternion(xyzw) q^k+ = q^k- +  1/2*XI(q^k-)*dAlpha^k+
    qk1    = qnormalize(qk1) ;            % Normalisation of Quaternions(xyzw) 
    biask1 = biask1 + delX(4:6,:);        % Update of Bias  Beta^k+ =  Beta^k- +  dBeta^k+
end

wk1 = w_B_BI_m - biask1; %  Estimated Angular Velocity without the bias

%% UPDATE THE VARIABLE FOR THE NEXT TIME STEP
qk    = qk1;         % Quaternions(xyzw)  q^k1- =  q^k+ 
biask = biask1;
wk    = wk1;
Pk    = Pk1;
% [wk1,qk1,biask1,Pk1]

w_B_BI_f = wk1;
q_B_I_f  = qinvert(qk1,'xyzw'); % Quaternions(wxyz) 
R_B_I_f  = q2R(q_B_I_f,'wxyz','tsf');
e_B_I_f  = R2eul(R_B_I_f,'ZYX');  % Euler Angles for Transformation from Inertia to Body

output(1:3,1)    = w_B_BI_f;
output(4:6,1)    = biask1;
output(1:6,2:7)  = Pk1;
output(1:4,8)    = q_B_I_f;
output(1:3,9)    = e_B_I_f;
end

