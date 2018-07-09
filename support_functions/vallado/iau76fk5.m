close all
clear all
clc

%% INPUT
r_ITRF = [-1033.4793830;  7901.2952754;  6380.3565958]; % [km]
v_ITRF = [-3.225636520;  -2.872451450;   5.531924446];  % [km/s]
a_ITRF = [0.001;0.002;0.003];                           % [km/s^2]
%% UTC Time
UTC = datetime(2004,4,6,7,51,28.386009); % UTC Time

JD_UTC  = jd(UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second); % [days]
d_AT    = 32.0;         % [sec] From Astronomical Almanac 2004:K9
d_UT1   = -0.4399619;   % [sec] April 6 2004 (Source: IERS EOPCO4)

x_p     = -0.140682/3600.0*pi/180;    % [rad]
y_p     =  0.333309/3600.0*pi/180;    % [rad]
dPsi    = -0.052195/3600.0*pi/180;    % [rad]
dEps    = -0.003875/3600.0*pi/180;    % [rad]
LOD     =  0.0015563;                 % [sec]

UT1 = UTC + seconds(d_UT1);   % [date]
TAI = UTC + seconds(d_AT);    % [date]
TT  = TAI + seconds(32.184);  % [date]

T_TT   = jd2000(TT.Year,TT.Month,TT.Day,TT.Hour,TT.Minute,TT.Second)/36525;
JD_UT1 = jd(UT1.Year,UT1.Month,UT1.Day,UT1.Hour,UT1.Minute,UT1.Second);


fprintf(1, '\n ============== convert various coordinate systems from ecef =================== \n');
        fprintf(1, 'ITRF          IAU-76/FK5   %14.7f %14.7f %14.7f', r_ITRF );
        fprintf(1, ' v %14.9f %14.9f %14.9f', v_ITRF );
        fprintf(1, ' a %14.9f %14.9f %14.9f\n', a_ITRF );
        
% ECEF to PEF - PEF = [polarm]*ITRF
[rpef, vpef, apef] = ecef2pef  (r_ITRF , v_ITRF, a_ITRF,x_p, y_p, T_TT );
        fprintf(1, 'PEF           IAU-76/FK5   %14.7f %14.7f %14.7f', rpef );
        fprintf(1, ' v %14.9f %14.9f %14.9f', vpef );
        fprintf(1, ' a %14.9f %14.9f %14.9f\n', apef );
        
% ECEF to TOD  - TOD = [sidereal]*[polarm]*ITRF
[rtod, vtod,atod] = ecef2tod(r_ITRF , v_ITRF, a_ITRF, T_TT, JD_UT1, LOD, x_p, y_p, 2, dPsi, dEps);
        fprintf(1, 'TOD 2 w corr  IAU-76/FK5   %14.7f %14.7f %14.7f', rtod );
        fprintf(1, ' v %14.9f %14.9f %14.9f', vtod );
        fprintf(1, ' a %14.9f %14.9f %14.9f\n', atod );

% ECEF to MOD - MOD = [nutation]*[sidereal]*[polarm]*ITRF
[rmod, vmod, amod] = ecef2mod  ( r_ITRF, v_ITRF, a_ITRF, T_TT, JD_UT1, LOD, x_p, y_p, 2, dPsi, dEps);
        fprintf(1, 'MOD 2 w corr  IAU-76/FK5   %14.7f %14.7f %14.7f', rmod );
        fprintf(1, ' v %14.9f %14.9f %14.9f', vmod );
        fprintf(1, ' a %14.9f %14.9f %14.9f\n', amod );      
        
% ECEF to ECI - GCRF = [precession]*[nutation]*[sidereal]*[polarm]*ITRF
[recig, vecig, aecig] = ecef2eci(r_ITRF , v_ITRF, a_ITRF, T_TT, JD_UT1, LOD, x_p, y_p, 2, dPsi, dEps);
        fprintf(1, 'GCRF 2 w corr IAU-76/FK5   %14.7f %14.7f %14.7f', recig );
        fprintf(1, ' v %14.9f %14.9f %14.9f', vecig );
        fprintf(1, ' a %14.9f %14.9f %14.9f\n', aecig );

% MOD2ECI - ECI= [precess]*MOD
        [recig, vecig, aecig] = mod2eci(rmod, vmod, amod, T_TT);
        fprintf(1, 'MOD->ECI                   %14.7f %14.7f %14.7f', recig );
        fprintf(1, ' v %14.9f %14.9f %14.9f', vecig );
        fprintf(1, ' a %14.9f %14.9f %14.9f\n', aecig );
        
%% IERS EOPCO4 Data (https://www.iers.org/IERS/EN/DataProducts/EarthOrientationData/eop.html)
%                    INTERNATIONAL EARTH ROTATION SERVICE
%                         EARTH ROTATION PARAMETERS
%                           EOP (IERS) C 04
% 
%              FORMAT(2X,I4,2X,A4,I3,2X,I5,2F9.6,F10.7,2X,F10.7,2X,2F9.6)
% ******************************************************************************
%   
%     Date         MJD     x        y       UT1-UTC       LOD       dPsi     dEpsilon
%                          "        "         s            s          "         "
%      (0h UTC)
%  2004  APR   1  53096-0.140110 0.320616-0.4336210   0.0007419  -0.051514-0.004442
%  2004  APR   2  53097-0.140593 0.323311-0.4344781   0.0009598  -0.051404-0.004538
%  2004  APR   3  53098-0.140693 0.325995-0.4355427   0.0011885  -0.051487-0.004522
%  2004  APR   4  53099-0.140819 0.328640-0.4368381   0.0014254  -0.051755-0.004414
%  2004  APR   5  53100-0.141058 0.330942-0.4383567   0.0015801  -0.052144-0.004266
%  2004  APR   6  53101-0.140689 0.333287-0.4399494   0.0015516  -0.052560-0.004138
%  2004  APR   7  53102-0.140038 0.336150-0.4414137   0.0013498  -0.052889-0.004087
%  2004  APR   8  53103-0.139788 0.339295-0.4426205   0.0010512  -0.053060-0.004109
%  2004  APR   9  53104-0.139651 0.342459-0.4435140   0.0007703  -0.053041-0.004171
%  2004  APR  10  53105-0.139123 0.345745-0.4441848   0.0005809  -0.052845-0.004257
