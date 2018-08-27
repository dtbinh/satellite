% This program is used to estimate the measurement variance
% and process noise variance using the adaptive filtering 
% approach presented in section 5.7.4. The model is given by a 
% first-order Markov process with time contant of 15 seconds 
% and sampling interval of 0.01 seconds. Averaged results are 
% shown over a 200 Monte Carlo run simulation.

% Optimal Estimation of Dynamic Systems by Crassidis and Junkins
% Adaptive Filter Example (not shown in book)

% Written by John L. Crassidis 6/04

% Other Required Routines: none

% Model
dt=0.01;t=[0:dt:1000]';m=length(t);
f=-1/15;g=1;h=1;d=0;
[phi,gam]=c2d(f,g,dt);

% Setup and True Variances
x0=10;xe=zeros(m,1);xe(1)=x0;
res=zeros(m,1);
r=0.01;q=0.1;
i20=0;

% Perform Monte Carlo Runs
m_monte=200;
r_est_monte=zeros(m_monte,1);q_est_monte=zeros(m_monte,1);

for j=1:m_monte,

% Display When Every 20th Point Is Reached
if (i20==20), 
 disp(sprintf('      Monte Carlo run has reached point %5i',j-1))
 i20=0;
end
i20=i20+1; 
    
% Truth and Measurements
y=dlsim(phi,gam,h,d,sqrt(q)*randn(m,1),x0);
ym=y+sqrt(r)*randn(m,1);

% Guess for Variances
r0=.1;q0=1;

% Solve Discrete-Time ARE and Gain
p=dare(phi',h',gam*q0*gam',r0);
gain=p*h'*inv(h*p*h'+r0);

% Kalman Filter
res(1)=ym(1)-xe(1);
for i = 1:m-1,
 xe(i+1)=phi*xe(i)+phi*gain*(ym(i)-h*xe(i));
 res(i+1)=ym(i+1)-xe(i+1);
end

% Autocorrelation and Correlation Coefficient
% (we must pick values after convergence) 
e0=res(1001:m-1);
e1=res(1002:m);
c0=1/(m-1)*sum(e0.*e0);
c1=1/(m-1)*sum(e1.*e0);
rho=c1/c0;

% Estimate for r (note: M = 1)
z_est=c1+h*phi*gain*c0;
r_est_monte(j)=c0-h*z_est;

% Estimate for q
omega_est=phi*(gain*c0*gain'-z_est*gain'-gain*z_est')*phi';
rhs=z_est*phi^(-1)*h'-h*phi^(1)*z_est-h*omega_est*phi^(-1)*h';
q_est_monte(j)=rhs/(h*gam^2*phi^(-1)*h');

end

% Average Results
r_true=r
r_est=mean(r_est_monte)
q_true=q
q_est=mean(q_est_monte)

% Plot Results
subplot(211)
plot(q_est_monte)
set(gca,'Fontsize',12);
ylabel('Q Estimate')

subplot(212)
plot(r_est_monte)
set(gca,'Fontsize',12);
xlabel('Run Number')
ylabel('R Estimate')