% function [utsec ] = hms2sec( hr,min,sec )
%
% This function converts hours, minutes and seconds into seconds from the
% beginning of the day.
%
%                utsec = hr * 3600.0 + min * 60.0 + sec;
%
% inputs                            
%    hr          - hours                          0 - 24
%    min         - minutes                        0 - 59
%    sec         - seconds                        0.0 - 59.99
%
% outputs      :
%    utsec       - seconds                        0.0 .. 86400.0
% 
% coupling      :
%    none.
%
% author        : rusty     -initial     05 jul 2018
%
% revisions
%                -

function utsec = hms2sec( hr,min,sec )

        utsec = hr * 3600.0 + min * 60.0 + sec;
end
