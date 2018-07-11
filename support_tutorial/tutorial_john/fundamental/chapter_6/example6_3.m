% This example shows a simulation to calibrate a three-axis magnetometer
% using the TWOSTEP algorithm and the centered solution.

% Fundamentals of Spacecraft Attitude Determination and Control by Markley and Crassidis
% Example 6.3

% Written by John L. Crassidis 9/13

% Other Required Routines: trmm_data.mat

% Get Measurements
dt=10;
load trmm_data
t=[0:dt:28800]';m=length(t);
ar=mag_i/10;
ctrue=[.5 .3 .6]'*10;
sigm=0.05;
d=[0.05 0.05 0.05;0.05 0.1 0.05;0.05 0.05 0.05];

% Monte Carlo Runs
num_mc=1000;i100=0;
x_lin=zeros(num_mc,9);
x_non=zeros(num_mc,9);
x_uf=zeros(num_mc,9);
x_kf=zeros(num_mc,9);
for jjj = 1:num_mc,

% Display When Every 100th Point is Reached
if (i100==100), 
 disp(sprintf('      Monte Carlo has reached point %5i',jjj-1))
 i100=0;
end
i100=i100+1;

% Measurements    
bm=(ar+kron(ctrue',ones(m,1))+sigm*randn(m,3))*inv(eye(3)+d)';

% Dot Product for Attitude Independent Measurement
ym=bm(:,1).*bm(:,1)+bm(:,2).*bm(:,2)+bm(:,3).*bm(:,3)...
 -(ar(:,1).*ar(:,1)+ar(:,2).*ar(:,2)+ar(:,3).*ar(:,3));

% TWOSTEP
bbtrue=(eye(3)+d)*ctrue;
etrue=2*d+d*d;
xc=[bbtrue;diag(etrue);etrue(1,2);etrue(1,3);etrue(2,3)]*0;
clear xe
xe(1,:)=xc';

norm_dx=100;
i=1;
while (norm_dx > 1e-8),
emat=[xc(4) xc(7) xc(8);xc(7) xc(5) xc(9);xc(8) xc(9) xc(6)];
ee=kron(xc(1:3)',ones(m,1))*inv(eye(3)+emat)';
kk=[bm(:,1).^2 bm(:,2).^2 bm(:,3).^2 2*bm(:,1).*bm(:,2) 2*bm(:,1).*bm(:,3) 2*bm(:,2).*bm(:,3)];
h=[2*(bm-ee) -kk(:,1)+ee(:,1).^2 -kk(:,2)+ee(:,2).^2 -kk(:,3)+ee(:,3).^2 -kk(:,4)+2*ee(:,1).*ee(:,2) -kk(:,5)+2*ee(:,1).*ee(:,3) -kk(:,6)+2*ee(:,2).*ee(:,3)];  
ye=2*[bm(:,1)*xc(1)+bm(:,2)*xc(2)+bm(:,3)*xc(3)]...
   -xc(4)*kk(:,1)-xc(5)*kk(:,2)-xc(6)*kk(:,3)-xc(7)*kk(:,4)-xc(8)*kk(:,5)-xc(9)*kk(:,6)...
   -xc(1)*ee(:,1)-xc(2)*ee(:,2)-xc(3)*ee(:,3);
dy=ym-ye;
dx=inv(h'*h)*h'*dy;
norm_dx=norm(dx);
xc=xc+dx;
xe(i+1,:)=xc';i=i+1;
end

% Linear Batch Solution Using Centering (Approximate Solution)
ybar=0;
llbar=zeros(9,1);

for i = 1:m,
ybar=ybar+1/m*ym(i);
llbar=llbar+1/m*[2*bm(i,:)';-kk(i,:)'];
end

ytilde=ym-ybar;
ll=[2*bm -kk];
lltilde=ll-kron(llbar',ones(m,1));

res=0;
info=zeros(9);
for i = 1:m,
res=res+ytilde(i)*lltilde(i,:)';
info=info+lltilde(i,:)'*lltilde(i,:);
end
xe_lin=inv(info)*res;

% Show TWOSTEP and Linear Solutions
evec=xc(4:9);
emat=[evec(1) evec(4) evec(5);evec(4) evec(2) evec(6);evec(5) evec(6) evec(3)];
[uu,ss]=eig(emat);
ww=diag([-1+sqrt(1+ss(1,1)) -1+sqrt(1+ss(2,2)) -1+sqrt(1+ss(3,3))]);  
dd=uu*ww*uu';
bb=inv(eye(3)+dd)*xc(1:3);
xcc=[bb(1);bb(2);bb(3);dd(1,1);dd(2,2);dd(3,3);dd(1,2);dd(1,3);dd(2,3)];

evec=xe_lin(4:9);
emat=[evec(1) evec(4) evec(5);evec(4) evec(2) evec(6);evec(5) evec(6) evec(3)];
[uu,ss]=eig(emat);
ww=diag([-1+sqrt(1+ss(1,1)) -1+sqrt(1+ss(2,2)) -1+sqrt(1+ss(3,3))]);  
dd=uu*ww*uu';
bb=inv(eye(3)+dd)*xe_lin(1:3);
xee_lin=[bb(1);bb(2);bb(3);dd(1,1);dd(2,2);dd(3,3);dd(1,2);dd(1,3);dd(2,3)];

x_non(jjj,:)=xcc';
x_lin(jjj,:)=xee_lin';

end

x_non_sol=mean(x_non)
sig3_non=std(x_non)*3

x_lin_sol=mean(x_lin)
sig3_lin=std(x_lin)*3

clf
plot(x_lin(:,1:3))
set(gca,'fontsize',12)
xlabel('Run Number')
ylabel('Bias Estimates')

pause

plot(x_non(:,1:3))
set(gca,'fontsize',12)
xlabel('Run Number')
ylabel('Bias Estimates')


