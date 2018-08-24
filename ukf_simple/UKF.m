function output = UKF(y_meas)
global CONST
dt = CONST.dt;
Q = CONST.Q;
R = CONST.R;
kappa = CONST.kappa;

persistent xk Pxx_k

if isempty(xk)
    xk    = [0;0;0];
    Pxx_k = diag([0.001 0.001 0.001]);
end

Dx = size(xk,1);
Dy = size(y_meas,1);
NSig = 2*Dx+1;

sig_x = (chol((Dx+kappa)*(Pxx_k+CONST.Q)))'
xk*ones(1,NSig)
x_sig_k = xk*ones(1,NSig)+[zeros(Dx,1) sig_x -sig_x]

RK       = zeros(Dx,4);
x_sig_k1 = zeros(Dx,NSig);
y_sig_k1 = zeros(Dy,NSig);

for i=1:NSig
    % State Sigma Points Runge Kutta Iterative Method
    RK(:,1) = fncA(x_sig_k(:,i));
    RK(:,2) = fncA(x_sig_k(:,i)+1/2*dt*RK(:,1));
    RK(:,3) = fncA(x_sig_k(:,i)+1/2*dt*RK(:,2));
    RK(:,4) = fncA(x_sig_k(:,i)+    dt*RK(:,3));
    x_sig_k1(:,i) = x_sig_k(:,i)+1/6*dt*RK*[1 2 2 1]';
    
    % Measurement Sigma Points
    y_sig_k1(:,i) = fncC(x_sig_k1(:,i));
end

% Weightage
W = ones(NSig,1)/(2*(Dx+kappa));
W(1,1) = kappa/(Dx+kappa);

% Mean Point
x_k1p = x_sig_k1*W;
y_k1p = y_sig_k1*W;

Pxx_k1p = Q;
Pyy_k1p = R;
Pxy_k1p = zeros(Dx,Dy);

for i=1:NSig
    xdif = x_sig_k1(:,i) - x_k1p;
    ydif = y_sig_k1(:,i) - y_k1p;
    Pxx_k1p = Pxx_k1p+xdif*xdif'*W(i,1);
    Pyy_k1p = Pyy_k1p+ydif*ydif'*W(i,1);
    Pxy_k1p = Pxy_k1p+xdif*ydif'*W(i,1);
end

t = get_param(CONST.model,'SimulationTime');

% Gain
K = Pxy_k1p/Pyy_k1p;

% Output
Pxx_k1 = Pxx_k1p - K*Pxy_k1p';
x_k1   = x_k1p + K*(y_meas-y_k1p);

xk = x_k1;
Pxx_k = Pxx_k1;

output = [x_k1,Pxx_k1];