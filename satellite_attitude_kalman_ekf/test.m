function output = test(input)
q_B_I = input(1:4,1); %wxyz q_B_I rotate of BODY frame from INERTIAL Frame (I to B)
e_B_I = input(5:7,1); % zyx (I to B)

e_B_I_1 = Quat2Euler(q_B_I,'wxyz');    % by thesis method xyz (I to B), wxyz (I to B) 
R_B_I = q2R(q_B_I,'wxyz','tsf'); % Transformation Matrix of Body Frame to Inertial Frame

e_B_I_2 = R2eul(R_B_I,'XYZ'); % xyz (I to B) verified ok


%%
q = qinvert(q_B_I,'wxyz');     % output 'xyzw'
qout = StarTracker(e_B_I_1,q);
q_OUT = qinvert(qout,'xyzw');  % output 'wxzy'


%% new

q = eul2q(e_B_I,'zyx','wxyz');
[~,I] = max(abs(q));
q_B_I_m = q/norm(q)*sign(q(I,1)/q_B_I(I,1));

%% OUTPUT
output(1:3,1) = e_B_I_1; % by thesis method xyz (I to B), wxyz (I to B) 
output(4:6,1) = e_B_I_2 ;  % Euler xyz Transformation from Inertial to Body Frame (I to B)
output(7:10,1) = q_OUT;
output(11:14,1) = q_B_I_m;
end