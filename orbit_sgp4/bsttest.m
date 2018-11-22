close all
clear all
clc
% script testmat.m
%
% This script tests the SGP4 propagator.

% Author: 
%   Jeff Beck 
%   beckja@alumni.lehigh.edu 

% Version Info: 
%   1.0 (051019) - Initial version from Vallado C++ version. 
%   1.0 (aug 14, 2006) - update for paper
%   2.0 (apr 2, 2007) - update for manual operations
%   3.0 (3 jul, 2008) - update for opsmode operation afspc or improved
%   3.1 (2 dec, 2008) - fix tsince/1440.0 in jd update

% these are set in sgp4init
global tumin mu radiusearthkm xke j2 j3 j4 j3oj2  
global opsmode

%%  ------------------------  implementation   --------------------------
%   Operation smode for 
% a - afspc 
% i - improved 
opsmode = 'i';

%   Type of Run
% c - compare 1 year of full satcat data
% v - verification run, requires modified elm file with
% m - maunual operation- either mfe, epoch, or dayof yr
typerun = 'm';

if (typerun == 'm')
% input mfe, epoch (YMDHMS), or dayofyr approach, m,e,d: ','s');
    typeinput = 'e';
else
    typeinput = 'e';
end;
% Earth Constants
% input constants 721, 72, 84 
whichconst = 84;

rad = 180.0 / pi;

%   ---------------- setup files for operation ------------------
%   input 2-line element set file
% infilename = '2018_10_05_CLYS_FM1.txt';
infilename = '2018_03_05_PICSAT.txt';
infile     = fopen(infilename, 'r');

if (infile == -1)
    fprintf(1,'Failed to open file: %s\n', infilename);
    return;
end
    
if (typerun == 'c')
    outfile = fopen('tmatall.out', 'wt');
else
    if (typerun == 'v')
        outfile = fopen('tmatver.out', 'wt');
    else
        outfile = fopen('tmat.out', 'wt');
    end
end

global idebug dbgfile
% ------------------------------------------------------------------------
% TLE: Read File
while (~feof(infile))
    
longstr1 = fgets(infile, 130);
        
    while ( (longstr1(1) == '#') && (feof(infile) == 0) )
        longstr1 = fgets(infile, 130);
    end

    if (feof(infile) == 0)

    longstr2 = fgets(infile, 130);

    fprintf('TLE:\n');
    fprintf('%s',longstr1(1:end-1));
    fprintf('%s',longstr2(1:end-1));
        if idebug
            catno = strtrim(longstr1(3:7));
            dbgfile = fopen(strcat('sgp4test.dbg.',catno), 'wt');
            fprintf(dbgfile,'this is the debug output\n\n' );
        end
    end
end
% ------------------------------------------------------------------------    
% TLE: convert the char string to sgp4 elements, includes initialization of sgp4
satrec = twoline2rv( whichconst, longstr1, longstr2);
% ------------------------------------------------------------------------
% TLE: print header
fprintf(outfile, ' satnum = %d xx\n', satrec.satnum);

fprintf('\n');
fprintf('Satellite   : %15d\n', satrec.satnum);
fprintf('Epoch Year  : %15d   [yrs]\n', satrec.epochyr);
fprintf('Epoch Time  : %15.3f [day](since 31.12. 00:00:00)\n', satrec.epochdays);
fprintf('JD (UT) Time: %15.6f [day]\n', satrec.jdsatepoch);
fprintf('Inclination : %15.6f [deg]\n', satrec.inclo*rad2deg);
fprintf('RAAN        : %15.6f [deg]\n', satrec.nodeo*rad2deg);
fprintf('AOP         : %15.6f [deg]\n', satrec.argpo*rad2deg);
fprintf('Mean Anomaly: %15.6f [deg]\n', satrec.mo*rad2deg);
fprintf('Eccentricity: %15.6f [ - ]\n', satrec.ecco);
fprintf('Mean Motion : %15.6f [rad/s]\n', satrec.no/60);
fprintf('BSTAR       : %15d [   ]\n', satrec.bstar);

fprintf('\nTEMED\n');

fprintf(' %8s %10s %10s %10s %9s %9s %9s %5s %3s %3s %2s:%2s:%9s | %9s | %14s | %9s %9s %9s %9s | %10s %10s %10s %10s %10s %10s\n',...
't[min]','ro(1)','ro(2)','ro(3)','vo(1)','vo(2)','vo(3)','year','mth','dd','hh','mm','ss','sat.argp','a', 'ecc','e(1)','e(2)','e(3)', 'incl', 'node', 'argp',' nu', 'm', 'arglat');


% ------------------------------------------------------------------------
%  SGP4: input start stop times manually
%    year = 2018;
%    mon  = 10;
%    day  = 5;
%    hr   = 2;
%    min  = 55;
%    sec  = 57;
   
   year = 2018;
   mon  = 3;
   day  = 5;
   hr   = 17;
   min  = 25;
   sec  = 55;
   jdstart = jday( year,mon,day,hr,min,sec );

   year = 2018;
   mon  = 3;
   day  = 5;
   hr   = 18;
   min  = 25;
   sec  = 56;
   jdstop = jday( year,mon,day,hr,min,sec );
   
   startmfe = (jdstart - satrec.jdsatepoch) * 1440.0;
   stopmfe  = (jdstop - satrec.jdsatepoch) * 1440.0;
   deltamin = 1;
           
% ------------------------------------------------------------------------
% SGP4: call the propagator to get the initial state vector value
[satrec, ro ,vo] = sgp4(satrec,  0.0);

% check so the first value isn't written twice
tsince = startmfe;
if ( abs(tsince) > 1.0e-8 )
    tsince = tsince - deltamin;
end
% ------------------------------------------------------------------------
% loop to perform the propagation
i = 0;

while ((tsince < stopmfe) && (satrec.error == 0))
    i=i+1;
    tsince = tsince + deltamin;

    if(tsince > stopmfe)
        tsince = stopmfe;
    end

    [satrec, ro, vo] = sgp4(satrec,  tsince);
    if (satrec.error > 0)
       fprintf(1,'# *** error: t:= %f *** code = %3i\n', tsince, satrec.error);
    end  
    
    jd = satrec.jdsatepoch + tsince/1440.0;
    [year,mon,day,hr,minute,sec] = invjday ( jd );
    
    satrec.argp = (((satrec.argpdot*tsince*60+satrec.argpo)));
    
    fprintf(' %8.3f %10.4f %10.4f %10.4f %9.6f %9.6f %9.6f %5i %3i %3i %2i:%2i:%9.6f | %9.4f | ',...
        tsince,ro(1),ro(2),ro(3),vo(1),vo(2),vo(3),year,mon,day,hr,minute,sec, satrec.argp*180/pi );
    
    [p,a,ecc,e,incl,node,argp,nu,m,arglat,truelon,lonper ]= rv2coe (ro,vo,mu);
    etotal(i,:) = e; 
    
    fprintf('%14.6f | %9.6f %9.6f %9.6f %9.6f | %10.5f %10.5f %10.5f %10.5f %10.2f  %10.2f\n',...
                            a, ecc,e(1),e(2),e(3), incl*180/pi, node*180/pi, argp*180/pi, nu*180/pi, m*rad, arglat);

  
end 
            
if (idebug && (dbgfile ~= -1))
    fclose(dbgfile);
end
fclose(infile);
fclose(outfile);
% ------------------------------------------------------------------------
% TEMED to ECEF
rteme = ro';
vteme = vo';
ateme = [0;0;0];

dut1 = 0;%0.4071728; %2009 Jan 1 Vallado
dat  = 0;%34.0;       %2009 Jan 1 Vallado

[ut1, tut1, jdut1, utc, tai, tt, ttt, jdtt, tdb, ttdb, jdtdb, tcg, jdtcg, tcb, jdtcb] ...
         = convtime ( year, mon, day, hr, min, sec, dut1, dat);
     
     
lod = 0;%0.87/1000;
xp  = 0;%0.0327*pi / (180.0*3600.0);
yp  = 0;%0.2524*pi / (180.0*3600.0);
eqeterms = 1;

[recef, vecef, aecef] = teme2ecef( rteme, vteme, ateme, ttt, jdut1, lod, xp, yp, eqeterms );

fprintf('\nECEF\n');
    fprintf(' %8s %10s %10s %10s %9s %9s %9s\n',...
    't[min]','recef(1)','recef(2)','recef(3)','vecef(1)','vecef(2)','vecef(3)');
fprintf(' %8.3f %10.4f %10.4f %10.4f %9.6f %9.6f %9.6f %5i %3i %3i %2i:%2i:%9.6f\n',...
                tsince,recef(1),recef(2),recef(3),vecef(1),vecef(2),vecef(3),year,mon,day,hr,minute,sec );
 % ------------------------------------------------------------------------    
 %  TEMED to COE
 fprintf('\nCOE\n');
 [p,a,ecc,e,incl,node,argp,nu,m,arglat,truelon,lonper ]= rv2coe (ro,vo,mu);
 
 fprintf(' %8s %8s %10s %10s %10s %10s %10s\n',...      
         'a [km]', 'ecc', 'incl [deg]', 'node [deg]', 'argp [deg]', 'nu [deg]', 'm [deg]');
         
 fprintf(' %8.3f %8.6f %10.3f %10.3f %10.3f %10.3f %10.3f %5i %3i %3i %2i:%2i:%9.6f \n',...
                a, ecc, incl*180/pi, node*180/pi, argp*180/pi, nu*180/pi, m*180/pi,year,mon,day,hr,minute,sec );
 % ------------------------------------------------------------------------           
 % ECEF to LATLON
 fprintf('\nLATLON\n');
 
 [latgc,latgd,lon,hellp] = ijk2ll ( recef );
 fprintf(' %8s %8s %8s\n',...
    'latgd','lon','hellp');
fprintf(' %8.4f %8.4f %8.4f\n',...
                latgd*rad2deg,lon*rad2deg,hellp);
 
            
            
fprintf('\nELIPTIC ORBIT\n');      
%-----------------------------------------
% Elliptic Orbits
ro = rteme;
vo = vteme;
ho = cross(ro,vo);
no = cross([0;0;1],ho);

%-----------------------------------------
% Inclination
io = acos(ho(3,1)/norm(ho));

fprintf('Inclination : %10.6f [deg]\n',io*180/pi);

%-----------------------------------------
% Right Ascension
raan = acos(no(1,1)/norm(no));

if (no(2,1)<0)
    raan = 2*pi - raan;
end

fprintf('RAAN        : %10.6f [deg]\n',raan*180/pi);
%-----------------------------------------
% True Anomaly
eo = 1/(mu)*((norm(vo)^2-(mu)/norm(ro))*ro - (dot(ro,vo))*vo );
fprintf('Eccentricity: %10.6f %10.6f %10.6f\n',eo);
fprintf('e           : %10.6f\n',norm(eo));
TA = acos(dot(eo,ro)/(norm(eo)*norm(ro)));

if  (dot(ro,vo)<0)
    TA = 2*pi - TA;
end

fprintf('True Anomaly: %10.6f [deg]\n',TA*180/pi);
%-----------------------------------------
% Argument of Perigee
aop = acos(dot(no,eo)/(norm(no)*norm(eo)));

if  (eo(3,1)<0)
    aop = 2*pi - aop;
end

fprintf('AOP         : %10.6f [deg]\n',aop*180/pi);

%-----------------------------------------
% Argument of Latitude
aol = acos(dot(no,ro)/(norm(no)*norm(ro)));
if  (ro(3,1)<0)
    aol = 2*pi - aol;
end
fprintf('AOL         : %10.6f [deg]\n',aol*180/pi);