% function [r_sun,rtasc_sun,decl_sun] = sun ( JD_UTC )
% ------------------------------------------------------------------------
%  This function calculates the geocentric equatorial position vector
%  the sun given the julian date.  this is the low precision formula and
%  is valid for years from 1950 to 2050.  accuaracy of apparent coordinates
%  is 0.01  degrees.  notice many of the calculations are performed in
%  degrees, and are not changed until later.  this is due to the fact that
%  the almanac uses degrees exclusively in their formulations.
%
%  inputs          description          range / units
%    jd          - julian date          days from 4713BC, Jan,01,12,00,00
%
%  outputs       :
%    r_sun       - ijk position vector of the sun au
%    rtasc_sun   - right ascension                rad
%    decl_sun    - declination                    rad
%
%  locals        :
%    meanlong_sun       - mean longitude
%    mean_anomaly_sun   - mean anomaly
%    eclp_lon           - ecliptic longitude
%    obl_eclp           - mean obliquity of the ecliptic
%    T_UT1              - julian centuries of ut1 from jan 1, 2000 12h
%    T_TDB              - julian centuries of tdb from jan 1, 2000 12h
%
%  coupling      :
%    none.
%
%  references    :
%    vallado       2007, 281, alg 29, ex 5-1
%
%  author        : vallado      719-573-2600   27 may 2002
%  revisions     : vallado     - fix mean lon of sun  7 mat 2004
%                  rusty       - initiate               


function [r_sun,rtasc_sun,decl_sun,eclp_lon,eclp_lat] = sun (JD_UT1)

% UT1 Julian Centuries
T_UT1   = (JD_UT1 -2451545.0 )/36525; % [day] from 2000, Jan,01,12,00,00

% Assumptions
T_TDB  = T_UT1;

% Mean Longitude of Sun in MOD frame
% mean_lon_sun = 280.460 + 36000.771*T_UT1;             % [deg]
mean_lon_sun = 280.460 + 36000.77*T_UT1;                % [deg]
mean_lon_sun = rem(mean_lon_sun,360);                   % [deg]

% Mean Anomaly of the Sun
% mean_anomaly_sun = 357.5291092 + 35999.05034*T_TDB    % [deg]
mean_anomaly_sun = 357.5277233 + 35999.05034*T_TDB;     % [deg]
mean_anomaly_sun = rem( mean_anomaly_sun,360 );         % [deg]
if ( mean_anomaly_sun < 0.0  )
    mean_anomaly_sun = mean_anomaly_sun+360;            % [deg]
end

mean_anomaly_sun = mean_anomaly_sun *deg2rad;           % [rad]


% Ecliptic Longitude 
eclp_lon = mean_lon_sun + 1.914666471*sin(mean_anomaly_sun)...
            + 0.019994643*sin(2.0*mean_anomaly_sun);   % [deg]
eclp_lon = rem(eclp_lon,360.0);                        % [deg]
eclp_lon = eclp_lon*deg2rad;                           % [rad]                                                     

% Ecliptic Latitude 
eclp_lat = 0;                                          % [deg]

% Obliquity of Ecliptic 
obl_eclp = 23.439291-0.0130042*T_TDB;                  % [deg] 
obl_eclp = obl_eclp*deg2rad;                           % [rad]

% Sun's Vector
R_sun  = 1.000140612-0.016708617*cos(mean_anomaly_sun)...
        -0.000139589*cos(2.0*mean_anomaly_sun);        % [AU]
r_sun  = [  R_sun*cos(eclp_lon);                       % [AU]
            R_sun*cos(obl_eclp)*sin(eclp_lon);         % [AU]
            R_sun*sin(obl_eclp)*sin(eclp_lon)];        % [AU]

% Right Ascension of the Sun
rtasc_sun = atan2( cos(obl_eclp)*tan(eclp_lon),1 );

% Quadrant check rtasc_sun is in the same quadrant as eclp_lon 
if ( eclp_lon < 0.0  )
    eclp_lon = eclp_lon + 2*pi;    
end

if ( abs( eclp_lon-rtasc_sun ) > pi*0.5  )
    rtasc_sun = rtasc_sun + 0.5 *pi*round( (eclp_lon-rtasc_sun)/(0.5 *pi));
end
        
% Declination of Sun        
decl_sun = asin( sin(obl_eclp)*sin(eclp_lon) );

end