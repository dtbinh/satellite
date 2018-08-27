% This example uses an extended Kalman filter to determine
% the position of a vehicle on the Earth from GPS pseudorange
% measurements. This program provides plots of the nonlinear 
% least squares and EKF position and velocity errors.

% Optimal Estimation of Dynamic Systems by Crassidis and Junkins
% Example 7.1

% Written by John L. Crassidis 9/03

% Other Required Routines: yumadata.mat, vecnorm.m 

% GPS Data from Yuma File
load yumadata
gpsdata=yuma;
gpsweek=gpsdata(13);
gpstime=gpsdata(4);

% Earth Parameters (mu, a and b are in meters)
we=7.2921151467e-5;
mu=398600.5e9;
e=0.0818;
a=6378137;
b=6356752.3142;

% Simulation Time Parameters
dt=10;tf=3600;t=[0:dt:tf]';m=length(t);i50=0;
sig3=zeros(m,4);sig3_nls=zeros(m,4);lam=zeros(m,1);phi=zeros(m,1);h=zeros(m,1);
pos_err=zeros(m,3);number_av=zeros(m,1);
pos_err_nls=zeros(m,3);pos_est=zeros(m,3);

% Measurement STD and Clock Bias in Meters
sigm=5;
bias=85000;
qbias=200;
drift=lsim(0,1,1,0,sqrt(qbias)*randn(m,1),t);

% Kalman Filter Parameters
ppos=1e6^2;pvel=100^2;pbias=100000^2;
pp=zeros(7);
pp(1,1)=ppos;pp(2,2)=ppos;pp(3,3)=ppos;
pp(4,4)=pvel;pp(5,5)=pvel;pp(6,6)=pvel;pp(7,7)=pbias;
sig3(1,:)=[pp(1,1) pp(2,2) pp(3,3) pp(7,7)].^(0.5)*3;
phi_s=eye(7);phi_s(1,4)=dt;phi_s(2,5)=dt;phi_s(3,6)=dt;
xe=zeros(m,7);
qv=0.00001;
gammaq=[dt^2/2*eye(3) zeros(3,1);dt*eye(4)];
qm=zeros(4);qm(1,1)=qv;qm(2,2)=qv;qm(3,3)=qv;qm(4,4)=qbias;

% Move 100 Km/Hr South from Washington, DC
deg_inc=((a+b)/2/1000)*2*pi/360;
south_inc=100/deg_inc/3600;
lambda=[38:south_inc:38+south_inc*m-south_inc]*pi/180;

% Main Loop
for i = 1:m-1,
    
% Display When Every 50th Point is Reached
if (i50==50), 
 disp(sprintf('      Program has reached point %5i',i-1))
 i50=0;
end
i50=i50+1;

% Get GPS SV Vectors
% Note the YUMA almanac gives the "longitude of the ascending
% node," which is referred to the Greenwich meridian. So, a 
% sidereal angle computation is not needed.

% Matrix of Stored ECEF SV Positions
r_ecef=zeros(28,3);

% Get Time of Applicability and Current Time
tapp=gpsdata(4);
tcurr=tapp+t(i);

for k = 1:28,

% Get Orbital Data
 j=(k-1)*13+1;
 ap=gpsdata(j+8);
 ma0=gpsdata(j+9);
 inc=gpsdata(j+4);
 sma=gpsdata(j+6)^2;
 ecc=gpsdata(j+2);
 an=gpsdata(j+7);
 anr=gpsdata(j+5);

% Mean Anomaly and Greenwich Longitude
 ma=ma0+sqrt(mu/(sma^3))*(tcurr-tapp);
 om=an+anr*(tcurr-tapp)-we*tcurr;
 
% Eccentric Anomaly (from "Celestial Mechanics", pg. 161) and Iteration
 big_e=ma+ecc*sin(ma)+(ecc^2/2)*sin(2*ma)+(ecc^3/24)* ...
      (9*sin(3*ma)-3*sin(ma))+(ecc^4/192)*(64 * ...
      sin(4*ma)-32*sin(2*ma));
 while abs(big_e-ecc*sin(big_e)-ma)>1e-14,
  big_e=big_e-(big_e-ecc*sin(big_e)-ma)/(1-ecc*cos(big_e));
 end
 
% True Anomaly, Argument of Latitude and Radius
 cta=(cos(big_e)-ecc)/(1-ecc*cos(big_e));
 sta=sqrt(1-ecc^2)* sin(big_e)/(1-ecc*cos(big_e));
 trueanon=atan2(sta,cta);
 arglat=trueanon+ap;
 radius=sma*(1-ecc*cos(big_e));
 
% Orbital Plane Positions
 x_prime=radius*cos(arglat);
 y_prime=radius*sin(arglat);
 
% ECEF Positions
 x_coord=x_prime*cos(om)-y_prime*cos(inc)*sin(om);
 y_coord=x_prime*sin(om)+y_prime*cos(inc)*cos(om);
 z_coord=y_prime*sin(inc);
 r_ecef(k,:)=[x_coord y_coord z_coord];
 
end

% Washington DC (note: h is given in meters)
phi=(-77)*pi/180;
h=0;
n_lam=a/sqrt(1-e^2*sin(lambda(i))^2);

% Position in ECEF
x_ecef=(n_lam+h)*cos(lambda(i))*cos(phi);
y_ecef=(n_lam+h)*cos(lambda(i))*sin(phi);
z_ecef=(n_lam*(1-e^2)+h)*sin(lambda(i));

% Vectors to SVs
rho=r_ecef-kron(ones(28,1),[x_ecef y_ecef z_ecef]);
rhomag=vecnorm(rho);
rho_n=rho./kron(rhomag,[1 1 1]);

% Find Available SVs (15 degree cutoff)
up=[cos(lambda(i))*cos(phi) cos(lambda(i))*sin(phi) sin(lambda(i))];
seenang=90-acos(rho_n(:,1)*up(1)+rho_n(:,2)*up(2)+rho_n(:,3)*up(3))*180/pi;
av=find(seenang > 15);
available_SV=av';
r_ecef_av=r_ecef(av,:);
ps=rhomag(av,:);
lm=length(ps);
number_av(i)=lm;

% Measurements 
psm=ps+sigm*randn(lm,1)+bias*ones(lm,1)+drift(i)*ones(lm,1);

% Kalman Filter Routine
xe(1,:)=[x_ecef y_ecef z_ecef 0 0 0 0];

rhoe=r_ecef_av-kron(ones(lm,1),[xe(i,1) xe(i,2) xe(i,3)]); 
pe=vecnorm(rhoe)+xe(i,7);
yres=psm-pe;
hh=[-rhoe./kron(vecnorm(rhoe),[1 1 1]) ones(lm,1)];
hh=[hh(:,1) hh(:,2) hh(:,3) zeros(lm,3) hh(:,4)];

% Update
gain=pp*hh'*inv(hh*pp*hh'+sigm^2*eye(lm));
xe(i,:)=xe(i,:)+(gain*yres)';
pp=[eye(7)-gain*hh]*pp;

% Propagate
xe(i+1,:)=(phi_s*xe(i,:)')';
pp=phi_s*pp*phi_s'+gammaq*qm*gammaq';

pos_err(i,:)=xe(i,1:3)-[x_ecef y_ecef z_ecef];
sig3(i+1,:)=abs([pp(1,1) pp(2,2) pp(3,3) pp(7,7)]).^(0.5)*3;

% Nonlinear Least Squares
max_nit=10;
clear re
re(1,:)=[0 0 0 0];
xx=re(1,:)';
dx=1e10;
ep_stop=1e-8;
ii=1;

while (norm(dx) > ep_stop), 
  rhoe=r_ecef_av-kron(ones(lm,1),[re(ii,1) re(ii,2) re(ii,3)]); 
  pe=vecnorm(rhoe)+re(ii,4);
  yres=psm-pe;
  h_nls=[-rhoe./kron(vecnorm(rhoe),[1 1 1]) ones(lm,1)];
  dx=inv(h_nls'*h_nls)*h_nls'*yres;
  xx=xx+dx;
  re(ii+1,:)=xx';
  if (ii == max_nit), break; end
  ii=ii+1;
end

pos_est(i,:)=xx(1:3)';
pos_err_nls(i,:)=xx(1:3)'-[x_ecef y_ecef z_ecef];
pcov_nls=inv(h_nls'*h_nls)*sigm^2;
sig3_nls(i,:)=diag(pcov_nls)'.^(0.5)*3;

end

% Finite Difference for NLS Velocity Estimate
vel_est=diff(pos_est)/dt;
vel_est(m,:)=vel_est(m-1,:);

% Plot Results
subplot(221)
plot(t/60,[-sig3(:,1) pos_err(:,1) sig3(:,1)]);
axis([0 60 -20 20])
set(gca,'fontsize',12)
set(gca,'Ytick',[-20 -10 0 10 20]);
set(gca,'Xtick',[0 10 20 30 40 50 60]);
ylabel('{\it x} (m)')
xlabel('Time (Min)')

subplot(222);
plot(t/60,[-sig3_nls(:,1) pos_err_nls(:,1) sig3_nls(:,1)]);
axis([0 60 -40 40])
set(gca,'fontsize',12)
set(gca,'Ytick',[-40 -20 0 20 40]);
set(gca,'Xtick',[0 10 20 30 40 50 60]);
ylabel('{\it x} (m)')
xlabel('Time (Min)')

subplot(223)
plot(t/60,xe(:,4))
set(gca,'fontsize',12)
axis([0 60 -4 4])
set(gca,'Ytick',[-4 -2 0 2 4]);
set(gca,'Xtick',[0 10 20 30 40 50 60]);
ylabel('{\it x} (m/s)')
xlabel('Time (Min)')

subplot(224)
plot(t/60,vel_est(:,1))
set(gca,'fontsize',12)
axis([0 60 -4 4])
set(gca,'Ytick',[-4 -2 0 2 4]);
set(gca,'Xtick',[0 10 20 30 40 50 60]);
ylabel('{\it x} (m/s)')
xlabel('Time (Min)')