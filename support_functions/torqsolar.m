% function y = torqsolar(R,A,V,w,rho,type)
% ------------------------------------------------------------------------
% This function calculates the torque due to aerodynamic disturbances based
% on either hughes theory or wertz theory. 
% 
% Inputs
%   R       - [m]   Matrix of Position Vectors of each dA
%   A       - [m^2] Matrix of Area Vector of each dA
%   S_B     - [m/s] Sun Vector in Body Frame
%   type    - [ ]   either wertz or hughes 
%
% ------------------------------------------------------------------------

function y = torqsolar(R,A,S_B,P_Sol,type)
if ~exist('type','var')
    type = 'wertz';
end

cosalpha = zeros(1,length(R(1,:)));
solar    = zeros(3,length(R(1,:)));

switch type
    case 'hughes'
        for i=1:1:length(R(1,:))
            N = A(:,i)/norm(A(:,i))                ;      % [] Surface Vector of dA
            cosalpha(:,i) = dot(N,S_B)/norm(N)/norm(S_B); % [] Scalar Quantity of Sun Angle to Surface
            solar(:,i) =  (cosalpha(:,i)>0)*cosalpha(:,i)*cross(R(:,i),S_B)*norm(A(:,i)) ; % Effective Area   
        end
        
        y =  P_Sol*sum(solar,2); % [Nm] Solar Torque by Hughes
        
    case 'wertz'
        
        for i=1:1:length(R(1,:))
            N = A(:,i)/norm(A(:,i))                ;      % [] Surface Vector of dA

            cosalpha(:,i) = dot(N,S_B)/norm(N)/norm(S_B); % [] Scalar Quantity of Sun Angle to Surface
            solar(:,i)    =  -(cosalpha(:,i)>0)*cosalpha(:,i)*cross(S_B,R(:,i))*norm(A(:,i)); % Effective Area
        end
         y =  P_Sol*sum(solar,2);  % [Nm] Solar Torque by Wertz

    otherwise
        
end

end