function output = actuatorReactionWheel(input)
tau   = input(1:3);  % [Nm] Desired Torque
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Perfect RW %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tau_m  = tau;        % [Nm] Output Torque

output = tau_m;
end