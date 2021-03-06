clear all
close all
clc
format long
%% VECTORS IN INERTIAL FRAME
v1i = [ 0.2673; 0.5345; 0.8018]; % Normalized Vector 1 in Inertial Frame  
v2i = [-0.3124; 0.9370; 0.1562]; % Normalized Vector 2 in Inertial Frame

%% EXACT VECTORS IN BODY FRAME
Rbi_exact = [ 0.5335 0.8080 0.2500; 
             -0.8080 0.3995 0.4330;
              0.2500 -0.4330 0.8660];

v1b_exact = Rbi_exact*v1i;         
v2b_exact = Rbi_exact*v2i;         

%% MEASURED VECTORS [+/- 5 degree]
v1b = [0.7814; 0.3751 ; 0.4987];
v2b = [0.6163; 0.7075 ; -0.3459];

%% TRIAD METHOD
[Rbi_triad,q_triad,J_triad] = triad(v1b,v2b,v1i,v2i);

Rerr_triad = Rbi_triad'*Rbi_exact;
q_error = dcm2q(Rerr_triad,'tsf','xyzw');
angle_error = acos(q_error(4))*2/pi*180;

fprintf('\n Triad:\t\t %8.8f | %.8f  %.8f  %.8f  %.8f',angle_error,q_error);
%% Q METHOD
[Rbi_qmethod,q_qmethod,J_qmethod] = qmethod(v1b,v2b,v1i,v2i);

Rerr_qmethod = Rbi_qmethod'*Rbi_exact;
q_error = dcm2q(Rerr_qmethod,'tsf','xyzw');
angle_error = acos(q_error(4))*2/pi*180;

fprintf('\n QMethod:\t %8.8f | %.8f  %.8f  %.8f  %.8f ',angle_error,q_error);
%% QUEST
[Rbi_quest,q_quest,J_quest] = quest(v1b,v2b,v1i,v2i,'chris');

Rerr_quest = Rbi_quest'*Rbi_exact;
q_error = dcm2q(Rerr_quest,'tsf','xyzw');
angle_error = acos(q_error(4))*2/pi*180;

fprintf('\n Quest (Chris):\t %8.8f | %.8f  %.8f  %.8f  %.8f',angle_error,q_error);

%% QUEST 2 (BST)
[Rbi_quest2,q_quest2,J_quest2] = quest(v1b,v2b,v1i,v2i,'buhl');
Rerr_quest2 = Rbi_quest2'*Rbi_exact;
q_error = dcm2q(Rerr_quest2,'tsf','xyzw');
angle_error = acos(q_error(4))*2/pi*180;

fprintf('\n Quest (Buhl):\t %8.8f | %.8f  %.8f  %.8f  %.8f',angle_error,q_error);


%% QUEST 2 (Crassidis)
[Rbi_quest3,q_quest3,J_quest3] = quest(v1b,v2b,v1i,v2i,'crassidis');
Rerr_quest3 = Rbi_quest3'*Rbi_exact;
q_error = dcm2q(Rerr_quest3,'tsf','xyzw');
angle_error = acos(q_error(4))*2/pi*180;

fprintf('\n Quest (John):\t %8.8f | %.8f  %.8f  %.8f  %.8f',angle_error,q_error);


fprintf('\n\n');