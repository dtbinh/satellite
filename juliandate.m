close all
clear all
clc
% [Vallado] Fundamentals of Astrodynamics and Applications 4th Edition -
% ALGORTHIM 14 Page 183

yr = 1970;
mo = 1;
d  = 1;
h  = 0; 
m  = 0;
s  = 0;
JD = 367*yr-floor((7*(yr+floor((mo+9)/12)))/4)+floor(275*mo/9)+d+1721013.5+((s/60+m)/60+h)/24;
since = JD;

yr = 2019;
mo = 6;
d  = 28;
h  = 8; 
m  = 0;
s  = 0;

JD = 367*yr-floor((7*(yr+floor((mo+9)/12)))/4)+floor(275*mo/9)+d+1721013.5+((s/60+m)/60+h)/24;

JD_ut1 = JD + 0.4071728/60/60/24;
JD_ut1 = JD;
fprintf("Date: %d/%d/%d %d:%d:%.2f\n",yr,mo,d,h,m,s);
fprintf("Julian Days: %.2f [days]\n",JD);
fprintf("Julian Days: %.2f [msec]\n",(JD*86400 - since*86400)*1000);
T_ut1 = (JD_ut1 - 2451545.0)/36525
fprintf("Julian Days: %.2f [secs]",(JD*86400 - since*86400));
mean_long = 280.460 + 36000.771*T_ut1

T_tdb = T_ut1;

sun_mean_anomaly = 357.5291092+35999.05034*T_tdb

ecliptic_long = mean_long+1.914666471*sin(deg2rad(sun_mean_anomaly))+0.019994643*sin(2*deg2rad(sun_mean_anomaly))

R = 1.000140612-0.016708617*cos(deg2rad(sun_mean_anomaly))-0.000139589*cos(deg2rad(sun_mean_anomaly))
e = 23.439291-0.0130042*T_tdb
ee = 0.016708617 - 0.000042037*T_tdb-0.0000001236*T_tdb^2

r = [R*cos(deg2rad(ecliptic_long));
     R*cos(deg2rad(e))*sin(deg2rad(ecliptic_long));
     R*sin(deg2rad(e))*sin(deg2rad(ecliptic_long))]
 
 
 

 R_km= r*149597870.7
 norm(R_km)