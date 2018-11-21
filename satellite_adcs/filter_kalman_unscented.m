function output = filter_kalman_unscented(input)
%% INPUT
w_B_BI_m   = input(1:3,1);   % Measured Angular Velocity (Gyro)

q_B_I_m(:,1) = input(4:7,1);
q_B_I_m(:,2) = input(8:11,1);
q_B_I_m(:,3)  = input(12:15,1);
S_B_m    = input(16:18,1);  % Measured Sun vector in Sensor Frame
B_B_m    = input(19:21,1);   % Measured Magnetic Field in Body Frame
S_I      = input(22:24,1);   % S Sun Vector in Inertial Frame since we have data of the Sun
B_I      = input(25:27,1);  % B Magnetic Vector in Inertial Frame since we have data of the Magnetic Field
T_m      = input(28:30,1);  % Measured Temperature, Referenced Temperaure, N/A
T_c      = input(31:33,1);  % Control Torque Desired
s_flag   = input(34:36,1);  % Sensor Flag [Gyro;ST1;ST2]

%% PARAMETERS
global CONST FLTR
t = get_param(CONST.model,'SimulationTime');
if (FLTR.dbukf) 
        fprintf('\n\n---------------UKF Time %.4f------------------',t);
end

dt      = CONST.dt;         % Sampling Time of Kalman Filter

sig_v   = CONST.sig_v;      % Noise Standard Deviation Attitude State
sig_u   = CONST.sig_u;      % Noise Standard Deviation Bias State

sig_mg  = CONST.sig_mg;     % Magnetometer Standard Deviation.
sig_st  = CONST.sig_st; 	% Star Tracker Standard Deviation (average)

t = get_param(CONST.model,'SimulationTime');

%% LOOP VALUES

persistent qk biask wk Pxx_k coefk;  % Variables that are retained in memory between calls to the function.

%% INITIAL VALUES
if isempty(qk)
    if (FLTR.dbukf) 
        fprintf('\nInitialisation');
    end
    
    % Initial Internal Loop
    qk    = FLTR.qk;          % Initial Quaternion (xyzw) for internal variable
    biask = FLTR.bk;          % Initial Bias Value from Lab Calibration
    coefk = zeros(3,1);       % Initial Temp Coeff
    wk    = FLTR.wk;          % Initial Value from measured gyro
    
    Pxx_k = FLTR.Pk_ukf;      % Initial Error Covariance 

    % Initial Output
    output(1:3,1)     = wk;
    output(4:6,1)     = biask;
    output(7:9,1)     = coefk;
    output(1:12,2:13) = zeros(12,12);
    output(1:4,14)    = qk; % Initial Quaternion (wxyz) for external output
    output(5:7,14)    = [0;0;0];   % Initial Euler Angles
    
    return;
end

% if norm(tau) > 5e-6
%    fprintf('\nError Covariance Reset');
%    Pxx_k = FLTR.Pk_ukf;
% end


lamda = FLTR.lamda;
a     = FLTR.alpha;

f = 2*(a+1);

%% SIGMA POINTS
if (FLTR.dbukf) 
    fprintf('\nSigma Points');
end

Qbar_k = dt/2*[(sig_v^2-1/6*sig_u^2*dt^2)*eye(3)      zeros(3)      ;
                        zeros(3)                   (sig_u^2)*eye(3) ];
 
% Reset x-K
x_k = [[0;0;0];biask];

Dx   = size(x_k,1);   % Size of State 6
Dy   = size(B_B_m,1); % Size of Measurement 3
NSig = 2*Dx+1 ;       % Size of Sigma Points 13

% Current Time Step Allocation
sig_x     = (chol((Dx + lamda)*(Pxx_k + Qbar_k)))' ;         % 6x6      
chi_sig_k = x_k*ones(1,NSig)+[zeros(Dx,1) sig_x -sig_x];    % 6x6

% Next Time Step Allocation
chi_sig_k1 = zeros(6,NSig);
y_sig_k1   = zeros(3,NSig); 
q_k1       = zeros(4,1);


for i = 1:NSig 

    del_q_k = delp2delq(chi_sig_k(1:3,i),a,f);
    q_sig_k = qmul(del_q_k,qk);
    w_sig_k = w_B_BI_m - chi_sig_k(4:6,i);
    
    % Propagate forward the Quaternion
    Mag_w = norm(w_sig_k);
    psik  = (sin(1/2*Mag_w*dt)/Mag_w)*w_sig_k;
    Omega = [ cos(1/2*Mag_w*dt)*eye(3)-smtrx(psik)         psik;
                               -psik'                 cos(1/2*Mag_w*dt)];
    q_sig_k1 = Omega*q_sig_k;
    
    % Save the first value as q_k1
    if i==1
       q_k1 = q_sig_k1 ;
    end

    del_q_k1 = qmul(q_sig_k1, [-q_k1(1:3,1); q_k1(4,1)]);

    % Propagate Sigma Points
    chi_sig_k1(1:3,i) = f*del_q_k1(1:3,1)/(a+del_q_k1(4,1));      % state sigma points
    chi_sig_k1(4:6,i) = chi_sig_k(4:6,i);                         % state sigma points 

    y_sig_k1_mg(:,i) = q2dcm(q_sig_k1)*B_I;  % Estimated measurement Sigma Point                           
    y_sig_k1(:,i) = f*q_sig_k1(1:3,1)/(a+q_sig_k1(4,1)); % dp

end


% Weights of Sigma Points
W = ones(NSig,1)/(2*(Dx+lamda));     % 1/2(6+1)) 
W(1,1) = lamda/(Dx+lamda);          % 1/(6+1)

% Mean Point
x_k1p = chi_sig_k1*W;     % State Variable Mean Point based on sigma point propagation
y_k1p = y_sig_k1*W;       % Measurement Mean Point based on sigma point propagation
y_k1p_mg = y_sig_k1_mg*W; % Measurement Mean Point (mg)

% Sensor Variance
R = sig_st(1)^2*eye(3);
R_mg = sig_mg^2*eye(3);

% Covariance 
Pxx_k1p = Qbar_k;

Pyy_k1p = R;
Pyy_k1p_mg = R_mg;

Pxy_k1p = zeros(Dx,Dy);
Pxy_k1p_mg = zeros(Dx,Dy);

for i = 1:1:NSig
    xdif = chi_sig_k1(:,i) - x_k1p;
    ydif =   y_sig_k1(:,i) - y_k1p;
    ydif_mg =   y_sig_k1_mg(:,i) - y_k1p_mg;
    
    Pxx_k1p = Pxx_k1p + xdif*xdif'*W(i,1); % Covariance of states
    Pyy_k1p = Pyy_k1p + ydif*ydif'*W(i,1); % Covariance of measurements
    Pyy_k1p_mg = Pyy_k1p_mg + ydif_mg*ydif_mg'*W(i,1); % Covariance of measurements
    
    Pxy_k1p = Pxy_k1p + xdif*ydif'*W(i,1); % Covariance of states
	Pxy_k1p_mg = Pxy_k1p_mg + xdif*ydif_mg'*W(i,1); % Covariance of states
end

%% GAIN AND UPDATE
if (FLTR.dbukf) 
    fprintf('\nGain and Update');
end

d_x_k1p = [0;0;0;0;0;0];
    

H = [eye(3) zeros(3)];

K      = Pxy_k1p/Pyy_k1p;            % Gain Update depends on covariance of meas/state and variance of state
Pxx_k1 = Pxx_k1p - K*Pxy_k1p';       % Error Covariance Update 
p_B_I_m = f*q_B_I_m(1:3,1)/(a+q_B_I_m(4,1)); % dp
res   = p_B_I_m - y_k1p;
d_x_k1p  = d_x_k1p + K*(res - H*d_x_k1p);             % dp           % [dp B] State Update

% 
% K_mg   = Pxy_k1p_mg/Pyy_k1p_mg;      % Gain Update
% Pxx_k1 = Pxx_k1 - K_mg*Pxy_k1p_mg';  % Error Covariance Update
% res    = B_B_m - y_k1p_mg;           % delta Magnetic Field
% x_k1   = x_k1 + K_mg*(res);          % State Update

x_k1 = x_k1p + d_x_k1p;

% Calculation of Updated Quaternion Error
del_q_k1 = delp2delq(x_k1(1:3,:),a,f);

% Propagate of State
qk1 = qmul(del_q_k1, q_k1);
qk1 = qnorm(qk1);

biask1 = x_k1(4:6,1);
wk1    = w_B_BI_m - biask1;

qk    = qk1;
biask = biask1;
Pxx_k = Pxx_k1;
wk    = wk1;
                   
%% OUTPUT
if (FLTR.dbukf) 
    fprintf('\nOutput');
end

w_B_BI_f = wk;                 % Angular Velocity of Body wrt to Inertial Frame 
q_B_I_f  = qk;                 % Quaternion (xyzw) from Inertia to Body
R_B_I_f  = q2dcm(q_B_I_f);     % Transformation Matrix from Inertia to Body
e_B_I_f  = dcm2eul(R_B_I_f);   % Euler Angles for Transformation from Inertia to Body

output(1:3,1)     = w_B_BI_f;
output(4:6,1)     = biask;
output(7:9,1)     = coefk;
output(1:12,2:13) = zeros(12,12);
output(1:4,14)    = q_B_I_f;
output(5:7,14)    = e_B_I_f;

end

