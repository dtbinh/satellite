function define(term)
if ~exist('term','var')
    fprintf('1asdasd1');
end

switch term
    
    case {'IJK','ijk'} 
        fprintf('Geocentric Equatorial Coordinate System');
    case {'GCRF','gcrf'} 
        fprintf('Geocentric Celestial Coordinate System (GCRF)\n');
        fprintf('GCRF is the standard inertial coordinate system for Earth.\n');
    case {'ITRF','itrf'} 
        fprintf('International Terrestrial Reference Frame (ITRF)\n');
        fprintf('ITRF is a geocentric coordinate frame fixed to the rotating \n');
        fprintf('earth. Its origin is at the center of the Earth and referenced \n');
        fprintf('at Greenwich Meridian.\n');
    case {'SEZ','sez'} 
        fprintf('Topocentric Horizon Coordinate system');
    case {'ECI','eci'} 
        fprintf('Earth Center Inertial');
    case {'ECEF','ecef'} 
        fprintf('Earth Centered Earth Fix');
    case {'GMST','gmst'} 
        fprintf('Greenwich Mean Sideral Time (GMST) \n');
        fprintf('Sideral Time of Greenwich Meridian with reference to Vernal Equinox\n');
    case {'MOD','mod'} 
        fprintf('Mean Equinox of Date (MOD) \n');
        fprintf('The plane intersection of ecliptic of date and mean equator \n');
        fprintf('of date. Ecliptic of Date is due to general precession which \n');
        fprintf('causes the equinox to move along the equator on a specific \n');
        fprintf('date. Mean Equator of Date is due to Earths rotation axis \n');
        fprintf('prcessing secularly about the North ecliptic pole, causing \n');
        fprintf('the equator to wobble about the ecliptic\n');
    
    case {'PEF','pef'} 
        fprintf('Polar Effect Frame (PEF) \n');
    case {'TOD','tod'} 
        fprintf('True Equator of Date (TOD) \n');
    
    % Date and Time
    case {'UTC','utc'} 
        fprintf('Coordinated Universal Time (UTC) \n'); 
    case {'UT1','ut1'} 
        fprintf('Universal Time (UT1) \n'); 
    case {'JD','jd'} 
        fprintf('Julian Date (JD) \n'); 
    case {'MJD','mjd'} 
        fprintf('Modified Julian Date (MJD) \n');
    case {'TDB','tdb'} 
        fprintf('Barycentric Dynamical Time (TDB) \n'); 
    case {'MJD','mjd'} 
        fprintf('Modified Julian Date (MJD) \n'); 
    
      
        
    
    otherwise
        fprintf('..');
end

fprintf('\n');
end
% This function returns the conversion from degree to radian
% inputs         
%   deg         - angle in degree         [deg]
%
% outputs       
%   rad         - angle in radian         [rad]
% 
% author        
%   rusty        - initial     05 jul 2018
%
% reference
%   
