% function [R_qst,q_qst,J_qst] = quest(v1b,v2b, v1i, v2i)
% ---------------------------------------------------------------------------
% This function takes in two known vectors in inertial frame and two
% measured vectors in reference frame using Quest Method. The ouput is a Rotation Matrix.
% 
% Inputs
%	v1b - measure vector 1 in body frame
%   v2b - measure vector 2 in body frame
%   v1i - known vector 1 in inertial frame
%   v2i - known vector 2 in inertial frame
% 
% Output
%   R_qst - Rotation Matrix
%   q_qst - Quaternion
%
% Revision
%   rusty - 30 aug 2018
%
% Reference
%   chris hall http://www.dept.aoe.vt.edu/~cdhall/courses/aoe4140/
%   crassidis Fundamentals Spacecraft Attitude Determination Control
% ----------------------------------------------------------------------

function [R_qst,q_qst,J_qst] = quest(v1b,v2b,v1i,v2i,questtype)

w1 = 1/2;
w2 = 1/2;

B = w1*(v1b*v1i')+w2*(v2b*v2i');
S = B + B';
Z = [B(2,3)-B(3,2);B(3,1)-B(1,3);B(1,2)-B(2,1)];
sigma = trace(B);

% Lamda Max 
        dot_b = dot(v1b,v2b);
        dot_i = dot(v1i,v2i);

        cross_b = cross(v1b,v2b);
        cross_i = cross(v1i,v2i);

        lambda_max = sqrt(0.5*0.5 + 0.5*0.5 + 2*0.5*0.5*(dot_b*dot_i+norm(cross_b)*norm(cross_i)));


switch questtype
    case 'chris'

    p = ((lambda_max+sigma)*eye(3) - S)^-1*Z;

    q = 1/sqrt(1+p'*p)*[p;1];

    case 'crassidis'

        % Adjudate
        AdjS = adj(S);

        %Rho
        rho = lambda_max + sigma;

        % X
        x = (AdjS + rho*S + rho*(rho-trace(S))*eye(3))*Z;
        gamma = rho^3-rho^2*trace(S)+trace(AdjS)*rho-det(S);

        q(1,1)  = x(1) /sqrt(gamma^2 + x'*x);
        q(2,1)  = x(2) /sqrt(gamma^2 + x'*x);
        q(3,1)  = x(3) /sqrt(gamma^2 + x'*x);
        q(4,1)  = gamma/sqrt(gamma^2 + x'*x);  
        
    case 'buhl'
        
        % Adjudate
        AdjS = adj(S);

        % Alpha and Beta
        alpha = lambda_max^2 - trace(B)^2 + trace(AdjS);
        beta  = lambda_max - trace(B);

        % Gamma
        gamma = alpha*(lambda_max + trace(B))-det(S);   

        % X
        x = (diag([alpha alpha alpha]) + beta*S + S^2)*Z;

        q(1,1)  = x(1) /sqrt(gamma^2 + x'*x);
        q(2,1)  = x(2) /sqrt(gamma^2 + x'*x);
        q(3,1)  = x(3) /sqrt(gamma^2 + x'*x);
        q(4,1)  = gamma/sqrt(gamma^2 + x'*x);  
end
    
    
    
    R_qst = q2dcm(q);
    q_qst = q;
    J_qst = w1*(1 - v1b'*R_qst*v1i) + w2*(1 - v2b'*R_qst*v2i);


end

