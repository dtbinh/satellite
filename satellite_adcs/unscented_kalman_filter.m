function output = unscented_kalman_filter(input)
%% INPUT
w_B_BI_m   = input(1:3,1);   % Measured Angular Velocity (Gyro)

q_B_I_m(:,1) = input(4:7,1);
q_B_I_m(:,2) = input(8:11,1);

S_S_m(:,1)   = input(12:14,1); % Measured Sun vector in Sensor Frame
S_S_m(:,2)   = input(15:17,1); % Measured Sun vector in Sensor Frame
S_S_m(:,3)   = input(18:20,1); % Measured Sun vector in Sensor Frame
S_S_m(:,4)   = input(21:23,1); % Measured Sun vector in Sensor Frame

B_B_m   = input(24:26,1);   % Measured Magnetic Field in Body Frame
S_I     = input(27:29,1);  % Sun Vector in Inertial Frame since we have data of the Sun
B_I     = input(30:32,1);  % B Magnetic Vector in Inertial Frame since we have data of the Magnetic Field
B_B     = input(33:35,1);  % B magnetic Vector in Body Frame
S_B     = input(36:38,1);  % Sun Vector in Body Frame
s_flag  = input(39:41,1);  % Sensor Flag [Gyro;ST1;ST2]


%% PARAMETERS
global CONST

dt      = CONST.dt;         % Sampling Time of Kalman Filter

sig_v   = CONST.sig_v;      % Noise Standard Deviation Attitude State
sig_u   = CONST.sig_u;      % Noise Standard Deviation Bias State
sig_w   = 1e-5   ;          % Noise Standard Deviation Angular Velocity State

sig_mg  = CONST.sig_mg;     % Magnetometer Standard Deviation

t = get_param(CONST.model,'SimulationTime');
%% LOOP VALUES

persistent qk biask wk Pxx_k coefk;  % Variables that are retained in memory between calls to the function.

%% INITIAL VALUES
if isempty(qk)
    
    % Initial Internal Loop
    qk    = [0;0;0;1];                    % Initial Quaternion (xyzw) for internal variable
    biask = [0;0;0];                % Initial Bias Value from Lab Calibration
    coefk = zeros(3,1);                   % Initial Temp Coeff
    wk    = [0;0;0];                      % Initial Value from measured gyro
    
    Pxx_k    = [(1)^2*eye(3)      zeros(3); 
                  zeros(3)  (3*pi/180)^2*eye(3)]; % Initial Error Covariance 
              
    % Initial Output
    output(1:3,1)     = wk;
    output(4:6,1)     = biask;
    output(7:9,1)     = coefk;
    output(1:12,2:13) = zeros(12,12);
    output(1:4,14)    = [0;0;0;1]; % Initial Quaternion (wxyz) for external output
    output(5:7,14)    = [0;0;0];   % Initial Euler Angles
    
    return;
end

lambda = 1;
a      = 1;

f = 2*(a+1);

%% SIGMA POINTS
Qbar_k = dt/2*[(sig_v^2-1/6*sig_u^2*dt^2)*eye(3)      zeros(3)         ;
                        zeros(3)                   (sig_u^2*dt)*eye(3) ];
 
% Reset x-K
x_k = [[0;0;0];biask];

Dx   = size(x_k,1);   % Size of State 6
Dy   = size(B_B,1); % Size of Measurement 3
NSig = 2*Dx+1 ;       % Size of Sigma Points 13

% Current Time Step Allocation
sig_x     = chol((Dx + lambda)*(Pxx_k + Qbar_k))' ;         % 6x6      
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
    
%     R_B_I  = q2xi(q_sig_k1)'*q2psi(q_sig_k1);

    y_sig_k1(:,i) = q2dcm(q_sig_k1)*B_I;  % Estimated measurement Sigma Point                           
%     y_sig_k1(:,i) = B_B;

end


% Weights of Sigma Points
W = ones(NSig,1)/(2*(Dx+lambda));     % 1/2(6+1)) 
W(1,1) = lambda/(Dx+lambda);          % 1/(6+1)

% Mean Point
x_k1p = chi_sig_k1*W;
y_k1p = y_sig_k1*W;

% Sensor Variance
R = sig_mg^2*eye(3);

% Covariance
Pxx_k1p = Qbar_k;
Pyy_k1p = R;
Pxy_k1p = zeros(Dx,Dy);

for i = NSig
    xdif = chi_sig_k1(:,i) - x_k1p;
    ydif =   y_sig_k1(:,i) - y_k1p;

    Pxx_k1p = Pxx_k1p + xdif*xdif'*W(i,1); % Covariance of states
    Pyy_k1p = Pyy_k1p + ydif*ydif'*W(i,1); % Covariance of measurements
    Pxy_k1p = Pxy_k1p + xdif*ydif'*W(i,1); % Covariance of states
end
    
% Gain and Update
K      = Pxy_k1p/Pyy_k1p;              % Gain Update
Pxx_k1 = Pxx_k1p - K*Pxy_k1p';    % Error Covariance Update
x_k1   = x_k1p + K*(B_B - y_k1p); % State Update

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

