function output = solar_torque(input)
S_B     = input(1:3);  % [] Sun Vector in the Body Frame
eclipse = input(4);    % [] Eclipse Status
global CONST;          % [] Global Constant
solarP = CONST.SolarP; % [N/m^2] solar wind pressure

[R,A] = sat_geo('2u'); % Obtain the Geometry of the satellite

for i=1:1:length(R(1,:))
    N = A(:,i)/norm(A(:,i))                ;      % [] Surface Vector of dA

    cosalpha(:,i) = dot(N,S_B)/norm(N)/norm(S_B); % [] Scalar Quantity of Sun Angle to Surface
    
    solar_hughes(:,i) =  (cosalpha(:,i)>0)*cosalpha(:,i)*cross(R(:,i),S_B)*norm(A(:,i)) ;
    solar_wertz(:,i)  =  -(cosalpha(:,i)>0)*cosalpha(:,i)*cross(S_B,R(:,i))*norm(A(:,i));
end

tau_s =  (eclipse==0)*solarP*sum(solar_hughes,2); % [Nm] Solar Torque by Hughes
tau_s =  (eclipse==0)*solarP*sum(solar_wertz,2);  % [Nm] Solar Torque by Wertz


output = tau_s;
end