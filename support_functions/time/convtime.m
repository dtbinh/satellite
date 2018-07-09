% function [ut1, tut1, jdut1, utc, tai, tt, ttt, jdtt, tdb, ttdb, jdtdb, tcg, jdtcg, tcb, jdtcb] ...
%         = convtime ( year, mon, day, hr, min, sec, dut1, dat )
% 
% This function finds the time parameters and julian century values for inputs
%  of utc or ut1. numerous outputs are found as shown in the local variables.
%  because calucations are in utc, you must include timezone if ( you enter a
%  local time, otherwise it should be zero.
%
%  inputs          description                    range / units
%    year        - year                           1900 .. 2100
%    mon         - month                          1 .. 12
%    day         - day                            1 .. 28,29,30,31
%    hr          - universal time hour            0 .. 23
%    min         - universal time min             0 .. 59
%    sec         - universal time sec (utc)            0.0  .. 59.999
%    timezone    - offset to utc from local site  0 .. 23 hr
%    dut1        - delta of ut1 - utc             sec
%    dat         - delta of tai - utc             sec
%
%  outputs       :
%    ut1         - universal time                 sec
%    tut1        - julian centuries of ut1
%    jdut1       - julian date (days only)           days from 4713 bc
%    utc         - coordinated universal time     sec
%    tai         - atomic time                    sec
%    tdt         - terrestrial dynamical time     sec
%    ttdt        - julian centuries of tdt
%    jdtt        - julian date (days only)           days from 4713 bc
%    tdb         - terrestrial barycentric time   sec
%    ttdb        - julian centuries of tdb
%    jdtdb       - julian date of tdb             days from 4713 bc
%    tcb         - celestial barycentric time     sec
%    tcg         - celestial geocentric time      sec
%    jdtdb       - julian date (days only)           days from 4713 bc
%
%  locals        :
%    hrtemp      - temporary hours                hr
%    mintemp     - temporary minutes              min
%    sectemp     - temporary seconds              sec
%    localhr     - difference to local time       hr
%    jd          - julian date of request         days from 4713 bc
%    me          - mean anomaly of the earth      rad
%
%  coupling      :
%    hms_2_sec   - conversion between hr-min-sec .and. seconds
%    jday        - find the julian date
% 
%  revision        
%    vallado     - author 719-573-2600              04 jun 2002
%    vallado     - add tcg, tcb, etc                06 oct 2005
%    vallado     - fix documentation for dut1       08 oct 2002
%    rusty       - remove frac                      05 jul 2018  
% 
%  references    :
%    vallado       2007, 201, alg 16, ex 3-7
%

function [ut1, tut1, jdut1, utc, tai, tt, ttt, jdtt, tdb, ttdb, jdtdb, tcg, jdtcg, tcb, jdtcb] ...
         = convtime ( year, mon, day, hr, min, sec, dut1, dat )

% JD - Julian Days
    jday = jd( year, mon, day, hr , min, sec );
        
% MJD - Modified Julian Days
    mjd  = jday - 2400000.5;
    mfme = hr*60.0 + min + sec/60.0;

% UTC - Coordinated Universal Time 
    utc  = hms2sec( hr, min, sec );

% UT1 - Universal Time
    ut1   = utc + dut1;
    [hrtemp, mintemp,sectemp] = sec2hms(  ut1 );
    jdut1 = jd( year,mon,day, hrtemp, mintemp, sectemp );
    tut1  = (jdut1 - 2451545.0  )/ 36525.0;

% TAI - Atomic Time
    tai= utc + dat;
    [hrtemp,mintemp,sectemp] = sec2hms(  tai );
    jdtai = jd( year,mon,day, hrtemp, mintemp, sectemp );
    
% TT - Terrestial Time
    tt   = tai + 32.184;   
    [hrtemp,mintemp,sectemp] = sec2hms( tt );
    jdtt = jd( year,mon,day, hrtemp, mintemp, sectemp);
    ttt  = (jdtt - 2451545.0  )/ 36525.0;

% TDB - Barycentric Dynamical Time
% usno circular approach 
        tdb = tt + 0.001657*sin(628.3076*ttt+6.2401) ...
               + 0.000022*sin(575.3385*ttt+4.2970) ...
               + 0.000014*sin(1256.6152*ttt+6.1969) ...
               + 0.000005*sin(606.9777*ttt+4.0212) ...
               + 0.000005*sin(52.9691*ttt+0.4444) ...
               + 0.000002*sin(21.3299*ttt+5.5431) ...
               + 0.000010*ttt*sin(628.3076*ttt+4.2490);  % USNO circ (14)
        [hrtemp,mintemp,sectemp] = sec2hms( tdb );
        jdtdb = jd( year,mon,day, hrtemp, mintemp, sectemp );
        ttdb  = (jdtdb - 2451545.0  )/ 36525.0;
        
    % vallado approach (extra digits)
    %        me= 357.5277233  + 35999.05034 *ttt;
    %        me= modulo( me,360.0  );
    %        me= me * deg2rad;
    %        tdb= tt + 0.001658  * sin(me) + 0.00001385 *sin(2.0 *me);
    %        [hrtemp,mintemp,sectemp] = sec2hms( tdb );
    %        jdtdb = jday( year,mon,day, hrtemp, mintemp, sectemp );
    %        ttdb= (jdtdb - 2451545.0  )/ 36525.0;
    % std approach (digits)
    %        me= 357.53  + 0.9856003 * (jdtt - 2451545.0);   
    %        me= modulo( me,360.0  );
    %        me= me * deg2rad;
    %        tdb1= tt + 0.001658  * sin(me) + 0.000014 *sin(2.0 *me);
    %        [hrtemp,mintemp,sectemp] = sec2hms( tdb1 );
    %        jdtdb1 = jday( year,mon,day, hrtemp, mintemp, sectemp );
    %        ttdb1= (jdtdb1 - 2451545.0  )/ 36525.0;
    % ast alm approach (2006)
    %        me= 357.53  + 0.98560028 * (jdtt - 2451545.0);   
    %        me= modulo( me,360.0  );
    %        me= me * deg2rad;
    %        dlje = 246.11 + 0.90255617*(jdtt - 2451545.0);
    %        tdb2= tt + 0.001658  * sin(me) + 0.000021 *sin(dlje);
    %        [hrtemp,mintemp,sectemp] = sec2hms( tdb2 );
    %        jdtdb2 = jday( year,mon,day, hrtemp, mintemp, sectemp );
    %        ttdb2 = (jdtdb2 - 2451545.0  )/ 36525.0;
    


% TCG - Geocentric Coordinate Time
% approx with tai
        tcg = tt + 6.969290134e-10*(jdtai - 2443144.5)*86400.0;  % AAS 05-352 (10) and IERS TN (104)
        [hrtemp,mintemp,sectemp] = sec2hms( tcg );
        jdtcg = jd( year,mon,day, hrtemp, mintemp, sectemp );
        tt2 = tcg-6.969290134e-10*(jdtcg-2443144.5003725)*86400.0;

% binomial approach with days
%        lg=6.969290134e-10*86400.0;
%        tcg1 = tt + (jdtt - 2443144.5003725)*(lg + lg*lg + lg*lg*lg);
% days from 77
%        jdttx = jday( year,mon,day, 0, 0, 0.0); 
%        ttx = tt/86400.0 + jdttx-2443144.5003725  % days from the 1977 epoch
%        tcg2 = (jdttx - 6.969290134e-10*2443144.5003725) / (1.0 - 6.969290134e-10) % days
%        tcg2 = (tcg2 - jdttx)*86400*86400;
% sec from 77
%        ttx = tt + (jdttx-2443144.5003725)*86400.0;  % s from the 1977 epoch
%        tcg3 = ttx / (1.0 - 6.969290134e-10); % s
%        tcg3 = tcg3 -(jdttx-2443144.5003725)*86400.0;
% check with tcg
%        tcg4 = tt + 6.969290134e-10*(jdtcg - 2443144.5003725)*86400.0;  % AAS 05-352 (10) and IERS TN (104)
%        [hrtemp,mintemp,sectemp] = sec2hms( tcg4 );
%        jdtcg4 = jday( year,mon,day, hrtemp, mintemp, sectemp );
%        tt2 = tcg4-6.969290134e-10*(jdtcg4-2443144.5003725)*86400.0;
%        difchk = tt2-tt
        
        
        tcbmtdb = 1.55051976772e-8*(jdtai - 2443144.5)*86400.0;  % sec, value for de405 AAS 05-352 (10) and IERS TN (104)?
        tcb = tdb + tcbmtdb;
        [hrtemp,mintemp,sectemp] = sec2hms( tcb );
        jdtcb = jd( year,mon,day, hrtemp, mintemp, sectemp );
