close all
clear 
clc
%% 
X = [1;0;0];
Y = [0;1;0];
Z = [0;0;1];

% Transformation/Rotation from Frame I to Frame B
angle  = 45/180*pi;
vector = [0;1;0];

q_wxyz_B_I = [   cos(angle/2);
            vector(1)*sin(angle/2);
            vector(2)*sin(angle/2);
            vector(3)*sin(angle/2)]; % q = [ eta eps1 eps2 eps3]
        
q_wxyz_B_I = q_wxyz_B_I/norm(q_wxyz_B_I)
q_xyzw_B_I = [  vector(1)*sin(angle/2);
                vector(2)*sin(angle/2);
                vector(3)*sin(angle/2);
                cos(angle/2)]; % q = [eps1 eps2 eps3 eta] 
            
q_xyzw_B_I = q_xyzw_B_I/norm(q_xyzw_B_I)

%% Transformation/Rotation from Frame B to Frame I
angle  = -45/180*pi;
vector = [0;1;0];

q_wxyz_I_B = [   cos(angle/2);
            vector(1)*sin(angle/2);
            vector(2)*sin(angle/2);
            vector(3)*sin(angle/2)]; % q = [ eta eps1 eps2 eps3]
q_wxyz_I_B = q_wxyz_I_B/norm(q_wxyz_I_B)
q_xyzw_I_B = [  vector(1)*sin(angle/2);
            vector(2)*sin(angle/2);
            vector(3)*sin(angle/2);
                cos(angle/2)]; % q = [eps1 eps2 eps3 eta]    
q_xyzw_I_B = q_xyzw_I_B/norm(q_xyzw_I_B)

% Quat2Euler(q_xyzw)*180/pi

%% ROTATION

V = X
% q2dcm('wxyz','rot',q_wxyz_I_B)*V
% q2dcm('wxyz','tsf',q_wxyz_I_B)*V % Transformation
% % 
R_B_I = q2dcm('xyzw','rot',q_xyzw_I_B)*V % Quaternion Rotation from B to I
R_I_B = q2dcm('xyzw','tsf',q_xyzw_I_B)*V % Quaternion Transformation from B to I

R_I_B = q2dcm('xyzw','rot',q_xyzw_B_I)*V
R_B_I = q2dcm('xyzw','tsf',q_xyzw_B_I)*V
return
Rquat(q_wxyz)*V % Rotation Matrix of vector from Frame A to Frame B
ATT(q_xyzw)*V  % Transformation Matrix of vector from Frame A expressed to Frame B



euler = [0;45/180*pi;0]
q_try = euler2q(euler(1),euler(2),euler(3))
Rquat(q_try)*V

eul2R(euler,'ZYX')*V

%% EULER ANGLE
% Body Frame Rotated from Inertial Frame
euler  = 45/180*pi;
axis = [0;1;0];
R_B_I = DCM(2,euler) % Transformation Matrix from inertial frame  to body frame
V_I   = Z;       % Vector in Inertial Frame
V_B   = R_B_I*V_I

%% R to QUATERNION
% [James_Diebel]
R = DCM(2,euler)

if R(2,2)>-R(3,3)&&R(1,1)>-R(2,2)&&R(1,1)>-R(3,3)
    q = 0.5*[         (1+R(1,1)+R(2,2)+R(3,3))^0.5            ;
              (R(2,3)-R(3,2))/((1+R(1,1)+R(2,2)+R(3,3))^0.5)  ;
              (R(3,1)-R(1,3))/((1+R(1,1)+R(2,2)+R(3,3))^0.5)  ;
              (R(1,2)-R(2,1))/((1+R(1,1)+R(2,2)+R(3,3))^0.5)  ];
    
elseif R(2,2)<-R(3,3)&&R(1,1)>R(2,2)&&R(1,1)>R(3,3)
    q = 0.5*[ (R(2,3)-R(3,2))/((1+R(1,1)-R(2,2)-R(3,3))^0.5)  ;
                      (1+R(1,1)-R(2,2)-R(3,3))^0.5            ;
              (R(1,2)+R(2,1))/((1+R(1,1)-R(2,2)-R(3,3))^0.5)  ;
              (R(3,1)+R(1,3))/((1+R(1,1)-R(2,2)-R(3,3))^0.5)  ];
    
elseif R(2,2)>R(3,3)&&R(1,1)<R(2,2)&&R(1,1)<-R(3,3)
    q = 0.5*[ (R(3,3)-R(1,3))/((1-R(1,1)+R(2,2)-R(3,3))^0.5)  ;      
              (R(1,2)+R(2,1))/((1-R(1,1)+R(2,2)-R(3,3))^0.5)  ;
                      (1-R(1,1)+R(2,2)-R(3,3))^0.5            ;
              (R(2,3)+R(3,2))/((1-R(1,1)+R(2,2)-R(3,3))^0.5)  ];
    
elseif R(2,2)<R(3,3)&&R(1,1)<-R(2,2)&&R(1,1)<R(3,3)
    q = 0.5*[ (R(1,2)-R(2,1))/((1-R(1,1)-R(2,2)+R(3,3))^0.5)  ;
              (R(3,1)+R(1,3))/((1-R(1,1)-R(2,2)+R(3,3))^0.5)  ;
              (R(2,3)+R(3,2))/((1-R(1,1)-R(2,2)+R(3,3))^0.5)  ;
                      (1-R(1,1)-R(2,2)+R(3,3))^0.5            ];
end
q
R2q('wxyz','dir',R)
return


%% EULER XYZ QUATERNION
close all
clear
clc

syms phi theta psi
qx = [cos(phi/2);sin(phi/2);0;0]
qy = [cos(theta/2);0;sin(theta/2);0]
qz = [cos(psi/2);0;0;sin(psi/2)]

%wxyz
qmatrix(qx,'wxyz')*qmatrix(qy,'wxyz')*qz


%% EULER ZYX QUATERNION
close all
clear
clc

syms phi theta psi
qx = [cos(phi/2);sin(phi/2);0;0]
qy = [cos(theta/2);0;sin(theta/2);0]
qz = [cos(psi/2);0;0;sin(psi/2)]

%wxyz
q = qmatrix(qz,'wxyz')*qmatrix(qy,'wxyz')*qx
