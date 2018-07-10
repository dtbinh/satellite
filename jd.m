% function jday = jd(yr,mo,d,h,m,s)
%
% The function otputs the julian date given the year, month, day, hour,
% min, sec. The output unit is in days.
% 
% inputs          description                    range / units
%    year        - year                           1900 .. 2100
%    mon         - month                          1 .. 12
%    day         - day                            1 .. 28,29,30,31
%    hr          - universal time hour            0 .. 23
%    min         - universal time min             0 .. 59
%    sec         - universal time sec             0.0 .. 59.999
%
%  outputs       :
%    jd          - julian date                    days from 4713 bc
%  references    :
%    vallado       2013 page 183

function jday = jd(yr,mo,d,h,m,s)


jday = 367*yr...
        - floor((7*(yr + floor((mo+9)/12)))/4)...
        + floor(275*mo/9)...
        + d+1721013.5...
        + ((s/60+m)/60+h)/24;

