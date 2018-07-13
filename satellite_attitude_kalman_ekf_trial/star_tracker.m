function output = star_tracker(input)
q_B_I   = input(1:4,1);  % True Rotation(wxyz) Matrix from Inertia to Body Frame
e_B_I_m = input(5:7,1);  % Measured Euler Angle from Inertia to Body
sig_m   = input(8:10,1); % Noise in euler unit

%% NOISY EULER TO QUATERNION MEASUREMENTS (Model1)
q_B_I_m = eul2q(e_B_I_m,'zyx','wxyz');

[~,I] = max(abs(q_B_I_m));
q_B_I_m_1 = q_B_I_m/norm(q_B_I_m)*sign(q_B_I_m(I,1)/q_B_I(I,1));


%% NOISY EULER TO QUATERNION MEASUREMENTS (Model2)
% q_B_I_m(:,i)  = qmult(q_B_I(:,i),[0.5*sig_st*[randn(1,1);randn(1,1);randn(1,1)];1]);
%     q_B_I_m(:,i)  = qnorm(q_B_I_m(:,i));
q_B_I_m_2 = qmult(qinvert(q_B_I,'wxyz'),[0.5*sig_m;1]);
q_B_I_m_2 = qinvert(q_B_I_m_2,'xyzw');
q_B_I_m_2 = qnorm(q_B_I_m_2);


output = [q_B_I_m_1;q_B_I_m_2];
end