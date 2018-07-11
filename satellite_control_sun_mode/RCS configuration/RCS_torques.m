% This script calculates the available torque from each of the four
% thrusters for the ESM design problem form the P6 notes & then determines
% the spacecraft body frame axis-by-axis maximum torques which would be
% available. This information can be used to set the RCS saturation limits
% in the SISO atittude and rate controller time domain response scripts.


% ------------------------------------------------------------------------- 
% Preliminaries
% ------------------------------------------------------------------------- 
clear variables
clear global
close all
clc

% ------------------------------------------------------------------------- 
% Load the Design Parameters
% ------------------------------------------------------------------------- 
% Spacecraft centre of mass in s/c frame
com           =  [0 0 0]';

% Thruster Position Matrix [m] - position is relative to s/c frame
% (each row corresponds to different thruster in the order [1 2 3 4])
TPM           = [-1.2	 1.2 -1.5
                -1.2	 1.2  1.5
                -1.2	-1.2  1.5
                -1.2	-1.2 -1.5]';

% Thruster Alignment Matrix [-] - that is direction thruster is pointing in s/c frame
% (each row corresponds to different thruster in the order [1 2 3 4])
TAM           = [-0.97815	-0.2079	0
                -0.97815	-0.2079	0
                -0.97815	0.2079	0
                -0.97815	0.2079	0]';

% Thrust per thruster [N]
F_thr         = [20 20 20 20];


% ------------------------------------------------------------------------- 
% Determine Thruster Torque Matrix (TTM) & thus maximum torque available
% about each body axis - report this result to command window
% ------------------------------------------------------------------------- 
% compute individual thruster torque vectors in body frame
for i = 1:length(TPM)
    TTM_nom(:,i)  = cross(TPM(:,i) - com, -TAM(:,i)) * F_thr(i);
end

% compute minimum & maximum torques about each axis
for i=1:3
    Taxis_max(i,:) = [ sum(TTM_nom(i,:).*(TTM_nom(i,:)<0)) sum(TTM_nom(i,:).*(TTM_nom(i,:)>0)) ];
end

% echo results to command window
Taxis_max


% ------------------------------------------------------------------------- 
% Plot body frame position & force  vectors
% ------------------------------------------------------------------------- 
% graph style
line_width        = 2;
line_colour       = 'g';

% create basic figure window
CreateBasicAxes(3, {'X_{body}','Y_{body}','Z_{body}'});

% add spacecraft body representation
sc_colour         = 0.6*[1 1 1];           % colour of main s/c body
sc_transparency   = 0.6;
sc_body_width     = [1.2 1.2 1.5];
body_vertices_x   = (2*[0 1 1 0 0 0; 1 1 0 0 1 1; 1 1 0 0 1 1; 0 1 1 0 0 0] - 1)*sc_body_width(1);
body_vertices_y   = (2*[0 0 1 1 0 0; 0 1 1 0 0 0; 0 1 1 0 1 1; 0 0 1 1 1 1] - 1)*sc_body_width(2);
body_vertices_z   = (2*[0 0 0 0 0 1; 0 0 0 0 0 1; 1 1 1 1 0 1; 1 1 1 1 0 1] - 1)*sc_body_width(3);
h_body            = patch(body_vertices_x, body_vertices_y, body_vertices_z, sc_colour);
set(h_body, 'FaceAlpha', sc_transparency)

% add vectors indicating RCS thrust vectors
for i=1:4
    pos_array  = [zeros(3,1) TPM(:,i)];
    dir_array  = [zeros(3,1) TAM(:,i)] + TPM(:,i)*[1 1];
    lbl_pos    = 1.1*dir_array(:,2);
    plot3(dir_array(1,:), dir_array(2,:), dir_array(3,:), 'Color', line_colour, 'LineWidth', line_width)
    text(lbl_pos(1), lbl_pos(2), lbl_pos(3), ['thr' num2str(i)], 'Color', 'w')
end

% plot com
plot3(com(1), com(2), com(3),'marker', '+', 'markersize',5,'Linewidth',2,'color',line_colour)
text(com(1)+0.05, com(2)+0.05, com(3)+0.05, 'com', 'Color', 'w')
