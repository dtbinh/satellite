function tau_s = solar_torque(R,A,S_B,eclipse,solarP,type)
% R - [m]   Matrix of Position Vectors of each dA
% A - [m^2] Matrix of Area Vector of each dA

if ~exist('type','var')
    type = 'wertz';
end

for i=1:1:length(R(1,:))
    N = A(:,i)/norm(A(:,i))                ;      % [] Surface Vector of dA

    cosalpha(:,i) = dot(N,S_B)/norm(N)/norm(S_B); % [] Scalar Quantity of Sun Angle to Surface
    
    solar_hughes(:,i) =  (cosalpha(:,i)>0)*cosalpha(:,i)*norm(A(:,i))*cross(R(:,i),S_B) ;
    solar_wertz(:,i)  =  -(cosalpha(:,i)>0)*cosalpha(:,i)*cross(S_B,R(:,i))*norm(A(:,i));
end

switch type
    case 'hughes'
        tau_s =  (eclipse==0)*solarP*sum(solar_hughes,2); % [Nm] Solar Torque by Hughes
    case 'wertz'
        tau_s =  (eclipse==0)*solarP*sum(solar_wertz,2);  % [Nm] Solar Torque by Wertz
end