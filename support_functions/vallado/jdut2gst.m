%  gst = jdut2gst(jdut1) 
%
%  This function finds the greenwich sidereal time (iau-82), given the
%  julian days of UT1
% 
%  inputs          description                    range / units
%    jdut1       - julian date of ut1             days from 4713 bc
%
%  outputs       :
%    gst         - greenwich sidereal time        0 to 2pi rad
%
%  locals        :GST
%    temp        - temporary variable for reals   rad
%    tut1        - julian centuries from the
%                  jan 1, 2000 12 h epoch (ut1)
%
%  author        : david vallado                  719-573-2600    7 jun 2002
%  editor        : rusty
%  references    : vallado       2013, 188, Eq 3-47

function gst = jdut2gst(jdut1)

T_UT1 = ( jdut1 - 2451545.0 ) / 36525.0; % [centuries] Julian Centuries

temp = - 6.2e-6 * T_UT1 * T_UT1 * T_UT1 + 0.093104 * T_UT1 * T_UT1  ...
       + (876600.0 * 3600.0 + 8640184.812866) * T_UT1 + 67310.54841; % [sec]

temp = rem( temp/240.0/180.0*pi,2.0*pi ); % [rad]


if ( temp < 0.0 )
    temp = temp + 2.0*pi;
end

gst = temp;

      
end
