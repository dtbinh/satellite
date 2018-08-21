% function qout = qchk(qcur,qpre,type)
% --------------------------------------------------------------------------
% This function checks and correct the quaternion for sign change
% 
% Input 
%     qin     -   quaternion input
%     type    -   quaternion type
% 
% Output 
%     qout    -   quaternion output
% 
% Revision
%     rusty   -   initial     20 aug 2018
% 
% Reference
%     rusty
%     
% -------------------------------------------------------------------------

function qout = qchk(qcur,qpre)

[~,I] = max(abs(qcur));

if (qcur(I,1)*qpre(I,1)< 0) 
	qout = -qcur;
else
    qout = qcur;
end

end