% function v = vtruncate(v,vmax)
% --------------------------------------------------------------------------
% This function outputs the truncated vector with the maximum allowable
% 
% Input 
%     v       -   vector to be truncated 
%     vmax    -   vector of maximum values
% 
% Output 
%     v       -   truncated vector
% 
% Revision
%     rusty   -   initial     05 aug 2018
% 
% Reference
%     rusty
%     
% -------------------------------------------------------------------------
function v = vtruncate(v,vmax)

for i=1:1:3
    if (v(i) > vmax(i)) || (v(i) < -vmax(i)) 
     tmp = abs(vmax(i)/v(i));
      for k=1:1:3
        v(k) = v(k) * tmp;
      end
    end
end


