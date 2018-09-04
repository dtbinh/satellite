% This program shows attitude determination for the TRMM satellite.
% It first determines position and attitude. Then it simulates TAM
% and Sun sensor measurements. Finally, attitude determination is done.


% Fundamentals of Spacecraft Attitude Determination and Control by Markley and Crassidis
% TRMM Attitude Deterination Example in Chapter 5
% Written by John L. Crassidis and F. Landis Markley, 3/31/11

% Mu value
mu=398600.64;

% Other Required Routines: crossm, ecef2llh, eclipse, extract, i2b,
%                          mag_field, quat_err, solar, vecnorm

% TRMM Orbital Elements
% http://www.heavens-above.com/orbit.aspx?satid=25063&lat=040.52&lng=-74.450&loc=Serin+Observatory&alt=0&tz=EST
% TLE
% 1 25063U 97074A   11090.10673345  .00015646  00000-0  21779-3 0  6182
% 2 25063 034.9654 346.4609 0001584 266.6234 093.4312 15.
% Epoch (UTC):  02:33:41, Thursday, March 31, 2011 
% Eccentricity:  0.0001584 
% Inclination:  34.9654� 
% Perigee height:  398 km 
% Apogee height:  400 km 
% Right Ascension of ascending node:  346.4609� 
% Argument of perigee:  266.6234� 
% Revolutions per day:  15.56019969 
% Mean anomaly at epoch:  93.4312� 
% Orbit number at epoch:  76170 

% Time
dt=10;
tf=28800;
t=[0:dt:tf]';
length_t=length(t);

% Date
yr  = 2011;
mth = 3;
day = 31;
hr  = 2;
min = 32;
sec = 41;

% Orbit Parameters
e   =0.0001584;
e_sqrt_plus_minus = sqrt((1+e)/(1-e));
inc = 34.9654*pi/180;
w   = 266.6234*pi/180;
big_omega = 346.4609*pi/180;
m0        = 93.4312*pi/180;
n_mean    = 15.56019969*2*pi/3600/24;
a         = (mu/n_mean^2)^(1/3);

% Attitude Matrix for Orbit Conversion from Perifocal to Inertial
r11 =  cos(big_omega)*cos(w)-sin(big_omega)*sin(w)*cos(inc);
r12 = -cos(big_omega)*sin(w)-sin(big_omega)*cos(w)*cos(inc);
r21 =  sin(big_omega)*cos(w)+cos(big_omega)*sin(w)*cos(inc);
r22 = -sin(big_omega)*sin(w)+cos(big_omega)*cos(w)*cos(inc);
r31 =  sin(w)*sin(inc);
r32 =  cos(w)*sin(inc);

% Pre-Allocate Space
pos = zeros(length_t,3);
vel = zeros(length_t,3);
q   = zeros(length_t,4);
qe  = zeros(length_t,4);
sig3= zeros(length_t,3);
qer = zeros(length_t,4);
q_alp = zeros(length_t,4);
alpha = zeros(length_t,1);

% True Rate
ang_vel=[zeros(length_t,1) -n_mean*ones(length_t,1) zeros(length_t,1)];

% Gyros
sigu=sqrt(10)*1e-10;
sigv=sqrt(10)*1e-7;
num_g=dt*[1 1];den_g=2*[1 -1];
[phi_g,gam_g,c_g,d_g]=tf2ss(num_g,den_g);
bias1 = dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(length_t,1),0.1*pi/180/3600/dt);
bias2 = dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(length_t,1),0.1*pi/180/3600/dt);
bias3 = dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(length_t,1),0.1*pi/180/3600/dt);
bias = [bias1 bias2 bias3];
wgm = ang_vel+sqrt(sigv^2/dt+1/12*sigu^2*dt)*randn(length_t,3)+bias;


% Tolerance for Newton Iterations
eps = 1e-10;
count = 0; 
count_e = 0;
count_u = 0;
max_iter = 100;

% Main Loop
for i=1:length_t
  
% Elements Approach for Position
% Kepler's Equation
 m       = m0+n_mean*(t(i)-t(1));
 delta_e = 10;
 big_e   = m;
 
 while abs(delta_e) > eps
  delta_e = (m-(big_e-e*sin(big_e)))/(1-e*cos(big_e));
  big_e   = big_e+delta_e;
  count   = count + 1;
  if count == max_iter, disp(' Maximum Number of Iterations Achieved'), break, end
 end
 
 count = 0;
 
% Perifocal Position
 rmag = a*(1-e*cos(big_e));
 nu   = 2*atan(e_sqrt_plus_minus*tan(big_e/2));
 x_per = a*(cos(big_e)-e);
 y_per =a*sqrt(1-e^2)*sin(big_e);
 x_per_dot = -sqrt(mu*a)/rmag*sin(big_e);y_per_dot=sqrt(mu*a*(1-e^2))/rmag*cos(big_e);

% Inertial Position 
 pos(i,1) = r11*x_per+r12*y_per;
 pos(i,2) = r21*x_per+r22*y_per;
 pos(i,3) = r31*x_per+r32*y_per;
 vel(i,1) = r11*x_per_dot+r12*y_per_dot;
 vel(i,2) = r21*x_per_dot+r22*y_per_dot;
 vel(i,3) = r31*x_per_dot+r32*y_per_dot;

% Get Attitude
 lz = -pos(i,:)/norm(pos(i,:));
 ly = -cross(pos(i,:),vel(i,:))/norm(cross(pos(i,:),vel(i,:)));
 lx = cross(ly,lz);
 true_att = [lx;ly;lz]; 
 
 if i==1;
  q(1,:) = extract(true_att);
 else
     
  vel_norm = norm(ang_vel(i,:));
  co  = cos(0.5*vel_norm*dt);si=sin(0.5*vel_norm*dt);
  n1  = ang_vel(i,1)/vel_norm;n2=ang_vel(i,2)/vel_norm;n3=ang_vel(i,3)/vel_norm;
  qw1 = n1*si;qw2=n2*si;qw3=n3*si;qw4=co;
  om  = [qw4  qw3 -qw2 qw1;-qw3  qw4  qw1 qw2;qw2 -qw1  qw4 qw3;-qw1 -qw2 -qw3 qw4];
  q(i,1:4) = (om*q(i-1,1:4)')';
 end
end

% Magnetic Field Generator and Body Observations
mag_i = mag_field(pos,yr,mth,day,hr,min,sec,10,dt,1);
mag_b = i2b(q,mag_i);

% Sun Reference and Eclipse
sun_i = solar(yr,mth,day,hr,min,sec,dt,tf);
sun_b = i2b(q,sun_i);

% Sensor Frame Transformation
t1 = [-0.5736    0  -0.8192;
       0.4096 0.866 -0.2868;
       0.7094  -0.5 -0.4967];
   
t2 = [-0.5736    0    0.8192;
      -0.4096  0.866 -0.2868;
      -0.7094  -0.5  -0.4967];
  
sun_sen1 = (t1*sun_b')';
sun_sen2 = (t2*sun_b')';

sun_avail1 = eclipse(pos,sun_sen1);
sun_avail2 = eclipse(pos,sun_sen2);

avail1     = find(sun_avail1==1);
avail2     = find(sun_avail2==1);

sun_avail = zeros(length_t,1);
sun_avail(avail1) = 1;
sun_avail(avail2) = 1;

% Sun Measurements
sig_sun = 0.05*pi/180;

% DSS 1
alp1  =-sun_sen1(:,1)./sun_sen1(:,3)+sig_sun*randn(length_t,1);
beta1 =-sun_sen1(:,2)./sun_sen1(:,3)+sig_sun*randn(length_t,1);
sun_sen1m = [alp1 beta1 -ones(length_t,1)]./kron([1 1 1],vecnorm([alp1 beta1 -ones(length_t,1)]));

% Get Signs Correct
sun_sen1m(:,1) = sign(sun_sen1(:,1)).*sign(sun_sen1m(:,1)).*sun_sen1m(:,1);
sun_sen1m(:,2) = sign(sun_sen1(:,2)).*sign(sun_sen1m(:,2)).*sun_sen1m(:,2);
sun_sen1m(:,3) = sign(sun_sen1(:,3)).*sign(sun_sen1m(:,3)).*sun_sen1m(:,3);
sun_b1m  =(inv(t1)*sun_sen1m')';

% DSS 2
alp2  =-sun_sen2(:,1)./sun_sen2(:,3)+sig_sun*randn(length_t,1);
beta2 =-sun_sen2(:,2)./sun_sen2(:,3)+sig_sun*randn(length_t,1);
sun_sen2m =[alp2 beta2 -ones(length_t,1)]./kron([1 1 1],vecnorm([alp2 beta2 -ones(length_t,1)]));
% Get Signs Correct
sun_sen2m(:,1)=sign(sun_sen2(:,1)).*sign(sun_sen2m(:,1)).*sun_sen2m(:,1);
sun_sen2m(:,2)=sign(sun_sen2(:,2)).*sign(sun_sen2m(:,2)).*sun_sen2m(:,2);
sun_sen2m(:,3)=sign(sun_sen2(:,3)).*sign(sun_sen2m(:,3)).*sun_sen2m(:,3);
sun_b2m=(inv(t2)*sun_sen2m')';

% Three Axis Magnetometer (TAM) Measurements
sig_tam = 0.5*100;
mag_bm = mag_b+sig_tam*randn(length_t,3);

% Reduced Order Magnetic Field (Reference)
mag_ri = mag_field(pos,yr,mth,day,hr,min,sec,6,dt,1);

% Attitude Determination
for i = 1:length_t
 
 om_dss1 = [-crossm(sun_b1m(i,:)) sun_b1m(i,:)';-sun_b1m(i,:) 0];
 om_dss2 = [-crossm(sun_b2m(i,:)) sun_b2m(i,:)';-sun_b2m(i,:) 0];
 om_tam  = [-crossm(mag_bm(i,:)) mag_bm(i,:)';-mag_bm(i,:) 0];

 gam_dss =[crossm(sun_i(i,:)) sun_i(i,:)';-sun_i(i,:) 0];
 gam_tam =[crossm(mag_i(i,:)) mag_i(i,:)';-mag_i(i,:) 0];
 gam_tamr =[crossm(mag_ri(i,:)) mag_ri(i,:)';-mag_ri(i,:) 0];

  return
 % Good TAM Measuremmets Used Here 
 
 if sun_avail1(i) == 1 & sun_avail2(i) == 1
  k_mat = -1/sig_sun^2*om_dss1*gam_dss-1/sig_sun^2*om_dss2*gam_dss-1/sig_tam^2*om_tam*gam_tam;
  [v_eig,e_eig] = eig(k_mat);
  [ii,jj] = max(diag(e_eig));
  qe(i,:) = v_eig(:,jj)';
  p = -inv(2/sig_sun^2*crossm(sun_b1m(i,:))^2+1/sig_tam^2*crossm(mag_bm(i,:))^2);
 end
 
 if sun_avail1(i) == 1 & sun_avail2(i) == 0
  k_mat = -1/sig_sun^2*om_dss1*gam_dss-1/sig_tam^2*om_tam*gam_tam;
  [v_eig,e_eig] = eig(k_mat);
  [ii,jj] = max(diag(e_eig));
  qe(i,:) = v_eig(:,jj)';
  p = -inv(1/sig_sun^2*crossm(sun_b1m(i,:))^2+1/sig_tam^2*crossm(mag_bm(i,:))^2);
 end
 
 if sun_avail1(i) == 0 & sun_avail2(i) == 1
  k_mat = -1/sig_sun^2*om_dss2*gam_dss-1/sig_tam^2*om_tam*gam_tam;
  [v_eig,e_eig] = eig(k_mat);[ii,jj] = max(diag(e_eig));
  qe(i,:) = v_eig(:,jj)';
  p = -inv(1/sig_sun^2*crossm(sun_b1m(i,:))^2+1/sig_tam^2*crossm(mag_bm(i,:))^2);
 end
 
 if sun_avail1(i) == 0 & sun_avail2(i) == 0
  qe(i,:) = NaN;
  p = NaN;
 end
 
 sig3(i,:)=(diag(p).^(0.5))'*3*180/pi;
 
 % Corrupted TAM Measuremmets Used Here
 if sun_avail1(i) == 1 & sun_avail2(i) == 1
  k_mat=-1/sig_sun^2*om_dss1*gam_dss-1/sig_sun^2*om_dss2*gam_dss-1/sig_tam^2*om_tam*gam_tamr;
  [v_eig,e_eig]=eig(k_mat);[ii,jj]=max(diag(e_eig));
  qer(i,:)=v_eig(:,jj)';
 end

 if sun_avail1(i) == 1 & sun_avail2(i) == 0
  k_mat=-1/sig_sun^2*om_dss1*gam_dss-1/sig_tam^2*om_tam*gam_tamr;
  [v_eig,e_eig]=eig(k_mat);[ii,jj]=max(diag(e_eig));
  qer(i,:)=v_eig(:,jj)';
 end
 
 if sun_avail1(i) == 0 & sun_avail2(i) == 1
  k_mat=-1/sig_sun^2*om_dss2*gam_dss-1/sig_tam^2*om_tam*gam_tamr;
  [v_eig,e_eig]=eig(k_mat);[ii,jj]=max(diag(e_eig));
  qer(i,:)=v_eig(:,jj)';
 end
 
 if sun_avail1(i) == 0 & sun_avail2(i) == 0
  qer(i,:)=NaN;
 end
 
end

% Get Signs Correct
qe(:,1)=sign(qe(:,1)).*sign(q(:,1)).*qe(:,1);
qe(:,2)=sign(qe(:,2)).*sign(q(:,2)).*qe(:,2);
qe(:,3)=sign(qe(:,3)).*sign(q(:,3)).*qe(:,3);
qe(:,4)=sign(qe(:,4)).*sign(q(:,4)).*qe(:,4);

% Attitude Error Using Good TAM Measurements
qerr = quat_err(q,qe); 
erre = qerr(:,1:3)*2*180/pi;

% Attitude Error Using Corrupted TAM Measurements
qerr=quat_err(q,qer); 
errer=qerr(:,1:3)*2*180/pi;

% Simple Alpha Filter
q_alp(1,:) =qe(1,:);
alpha0 = 0.1;
alpha(1) = alpha0*norm(cross(mag_i(1,:)/norm(mag_i(1,:)),sun_i(1,:)))^2;

for i = 1:length_t-1,

% Propagate Quaternion
 ww=norm(wgm(i,:));
 co=cos(0.5*ww*dt);
 si=sin(0.5*ww*dt);
 n1=wgm(i,1)/ww;n2=wgm(i,2)/ww;n3=wgm(i,3)/ww;
 qw1=n1*si;qw2=n2*si;qw3=n3*si;qw4=co;
 om=[qw4  qw3 -qw2 qw1;-qw3  qw4  qw1 qw2;qw2 -qw1  qw4 qw3;-qw1 -qw2 -qw3 qw4];
 q_prop=(om*q_alp(i,:)');q_prop=q_prop/norm(q_prop);
 
% Alpha Filter
 if sun_avail(i+1) == 1
  alpha(i+1)=alpha0*norm(cross(mag_i(i,:)/norm(mag_i(i,:)),sun_i(i,:)))^2;
  q_alp(i+1,:)=(1-alpha(i+1))*q_prop'+alpha(i+1)*qe(i+1,:);
 else
  q_alp(i+1,:)=q_prop';
 end
  q_alp(i+1,:)=q_alp(i+1,:)/norm(q_alp(i+1,:));
 
end
 
% Attitude Error Using Corrupted TAM Measures and Alpha Filter
qerr=quat_err(q,q_alp);errea=qerr(:,1:3)*2*180/pi;

% Plot Orbit
clf
plot3(pos(:,1),pos(:,2),pos(:,3));grid
set(gca,'fontsize',12)
xlabel('X (km)')
ylabel('Y (km)')
zlabel('Z (km)')

pause

% Get Sidereal Angle
d_2000=367*yr-floor(7*(yr+floor((mth+9)/12))/4)+floor(275*mth/9)+(hr+min/60+(sec+t)/3600)/24+day-730531.5;
theta=(280.46061837+360.98564736628*d_2000)*pi/180;

% Get ECEF Positions
r_ecef=[cos(theta).*pos(:,1)+sin(theta).*pos(:,2) -sin(theta).*pos(:,1)+cos(theta).*pos(:,2) pos(:,3)];

% Get Latitude, Longitude and Height
[lat,long,height]=ecef2llh(r_ecef);

load topo
imageHndl=imagesc(topo);
set(gca,'Ydir','normal','XLim',[0 360],'Ylim',[0 180])
colormap((topomap1+white)/2);
hold on;
set(gca,'fontsize',12)
set(gca,'Xtick',[0:30:360]')
set(gca,'XtickLabel',[0 30 60 90 120 150 180 150 120 90 60 30 0])
set(gca,'Ytick',[0:10:180]')
set(gca,'YtickLabel',[-90,-80,-70,-60,-50,-40,-30,-20,-10,0, ...
10 20 30 40 50 60 70 80 90])
grid
title('Geodetic Position on Earth')
xlabel('Longitude (Deg)')
ylabel('Latitude (Deg)')
plot(long,lat+90,'r.')
hold off;

pause

% Plot Attitude Errors Using Good TAM Measurements
subplot(311)
plot(t/60/60,-sig3(:,1),'r',t/60/60,erre(:,1),'b',t/60/60,sig3(:,1),'r')
set(gca,'fontsize',12)
axis([0 8 -1 1])
ylabel('Roll (Deg)')
subplot(312)
plot(t/60/60,-sig3(:,2),'r',t/60/60,erre(:,2),'b',t/60/60,sig3(:,2),'r')
set(gca,'fontsize',12)
axis([0 8 -0.5 0.5])
ylabel('Pitch (Deg)')
subplot(313)
plot(t/60/60,-sig3(:,3),'r',t/60/60,erre(:,3),'b',t/60/60,sig3(:,3),'r')
set(gca,'fontsize',12)
axis([0 8 -1 1])
ylabel('Yaw (Deg)')
xlabel('Time (Hr)')

pause

% Plot Attitude Errors Using Corrupted TAM Measurements
subplot(311)
plot(t/60/60,errer(:,1),'b')
set(gca,'fontsize',12)
axis([0 8 -2 2])
ylabel('Roll (Deg)')
subplot(312)
plot(t/60/60,errer(:,2),'b')
set(gca,'fontsize',12)
axis([0 8 -1 1])
ylabel('Pitch (Deg)')
subplot(313)
plot(t/60/60,errer(:,3),'b')
set(gca,'fontsize',12)
axis([0 8 -2 2])
ylabel('Yaw (Deg)')
xlabel('Time (Hr)')

pause

% Plot Alpha Gain
clf
plot(t/60/60,alpha)
set(gca,'fontsize',12)
axis([0 8 -0.05 0.15])
ylabel('Alpha')
xlabel('Time (Hr)')

pause

% Plot Attitude Errors Using Corrupted TAM Measurements Using Alpha
subplot(311)
plot(t/60/60,errea(:,1),'b')
set(gca,'fontsize',12)
axis([0 8 -0.1 0.1])
ylabel('Roll (Deg)')
subplot(312)
plot(t/60/60,errea(:,2),'b')
set(gca,'fontsize',12)
axis([0 8 -0.1 0.1])
ylabel('Pitch (Deg)')
subplot(313)
plot(t/60/60,errea(:,3),'b')
set(gca,'fontsize',12)
axis([0 8 -0.1 0.1])
ylabel('Yaw (Deg)')
xlabel('Time (Hr)')