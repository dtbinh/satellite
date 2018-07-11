function h_aeolus = Vis3DSpacecraft(scale)
% Vis3DAelus. Generates 3D image of Aeolus & adds to current figure. Used
% in conjunction with other Vis3D functions.
%
% S.P.Hardacre 13.12.04

% CVS VERSION INFORMATION
% $Revision: 1.1 $
% $Author: HARDAC_S $
% $Date: 2005/03/14 14:14:24 $
% $Source: W:\\AOCS\\Aeolus_CVS_repository/OSE/code/support_functions/outputs/plot/Vis3DAeolus.m,v $
% $Name:  $
% $State: Exp $

if ~exist('scale')
    scale = 1;
end

% -------------------------------------------------------------------------
% Setup S/C shape constants
% (dimensions are normalised w.r.t. unit s/c dimension)
% -------------------------------------------------------------------------
% s/c main body
sc_body_width        = scale*0.5;             % half width of s/c body, assumed to be rectangular box
sc_colour            = 0.6*[1 1 1];           % colour of main s/c body
sc_transparency      = 0.5;                   % transparency of s/c body

body_vertices_x = (2*[0 1 1 0 0 0; 1 1 0 0 1 1; 1 1 0 0 1 1; 0 1 1 0 0 0] - 1)*sc_body_width;
body_vertices_y = (2*[0 0 1 1 0 0; 0 1 1 0 0 0; 0 1 1 0 1 1; 0 0 1 1 1 1] - 1)*sc_body_width;
body_vertices_z = (2*[0 0 0 0 0 1; 0 0 0 0 0 1; 1 1 1 1 0 1; 1 1 1 1 0 1] - 1)*sc_body_width;
h_body          = patch(body_vertices_x, body_vertices_y, body_vertices_z, sc_colour);
set(h_body, 'FaceAlpha', sc_transparency)

% connecting cylinder to payload, aligned with Z axis
sc_cyl2_radius       = sc_body_width*0.9;     % radius in XY plane
sc_cyl2_height       = sc_body_width*0.5;     % along Z axis direction
sc_cyl2_z_offset     = sc_body_width;         % baseplate offset w.r.t XY plane
sc_cyl2_colour       = 'y';
sc_cyl2_transparency = sc_transparency;
sc_cyl2_mesh_points  = 8;

% payload dish
dish_radius          = sc_body_width;
dish_phi             = 20*pi/180*scale;
dish_base_offset     = [0 0 sc_body_width + sc_cyl2_height];
dish_colour          = 'y';
dish_transparency    = sc_transparency;
dish_mesh_points     = 20;
dish_tripod_linewidth= 1;
dish_tripod_colour   = 0*[1 1 1];

% payload baffle cylinder, connects to payload dish
sc_cyl1_radius       = sc_body_width;
sc_cyl1_height       = sc_body_width*2;
sc_cyl1_z_offset     = sc_body_width + sc_cyl2_height + 2*dish_radius/scale*(1-cos(dish_phi));
sc_cyl1_colour       = 'w';
sc_cyl1_transparency = sc_transparency;
sc_cyl1_mesh_points  = 20;

[x_cyl1 y_cyl1 z_cyl1] = cylinder(sc_cyl1_radius*[1 1], sc_cyl1_mesh_points);
h_cyl1                 = surf(x_cyl1, y_cyl1, sc_cyl1_height*z_cyl1 + sc_cyl1_z_offset, 'EdgeColor', 'none', ...
                              'FaceColor', sc_cyl2_colour, 'FaceAlpha', sc_cyl1_transparency, 'EdgeAlpha', sc_cyl1_transparency);


% solar arrays
sa_width             = sc_body_width*0.8;     % half width of solar arrays (short axis of array)
sa_length            = sc_body_width*2.4;     % length of each solar array panel (long axis of array, three panels per array)
sa_angle             = pi/4;                  % solar array roll angle
sa_colour            = 'b';                   % solar array colour
sa_transparency      = 0.5;                   % solar array transparency
sa_x_offset          = sc_body_width*(1 + 0.5);
sa_hinge_linewidth   = 3*scale;
sa_hinge_colour      = 0.6*[1 1 0];




% -------------------------------------------------------------------------
% define points for cylinder components of s/c body & add to figure
% -------------------------------------------------------------------------
% baffle cylinder - note don't show edges since it's supposed to be
% cylinder cross-section rather than polygon

% add edge to cylinder
temp_points = (0:1/sc_cyl1_mesh_points:1)*2*pi;
temp_x      = sc_cyl1_radius*cos(temp_points);
temp_y      = sc_cyl1_radius*sin(temp_points);
temp_z      = (sc_cyl1_height + sc_cyl1_z_offset)*ones(1,length(temp_points));
h_cyl1edge  = plot3(temp_x, temp_y, temp_z, 'k');

% add tripod
dish_tripod = [0 -sc_cyl1_radius sc_cyl1_z_offset; 0 0 sc_cyl1_z_offset+sc_cyl1_height];
h_tripod1   = plot3(dish_tripod(:,1), dish_tripod(:,2), dish_tripod(:,3), 'LineWidth', dish_tripod_linewidth, 'Color', dish_tripod_colour);

dish_tripod = [+sc_cyl1_radius*sin(120*pi/180) -sc_cyl1_radius*cos(120*pi/180) sc_cyl1_z_offset; 0 0 sc_cyl1_z_offset+sc_cyl1_height];
h_tripod2   = plot3(dish_tripod(:,1), dish_tripod(:,2), dish_tripod(:,3), 'LineWidth', dish_tripod_linewidth, 'Color', dish_tripod_colour);

dish_tripod = [-sc_cyl1_radius*sin(120*pi/180) -sc_cyl1_radius*cos(120*pi/180) sc_cyl1_z_offset; 0 0 sc_cyl1_z_offset+sc_cyl1_height];
h_tripod3   = plot3(dish_tripod(:,1), dish_tripod(:,2), dish_tripod(:,3), 'LineWidth', dish_tripod_linewidth, 'Color', dish_tripod_colour);


% payload connection cylinder - show edges since it is (roughly) a
% hexagonal cross-section)
[x_cyl2 y_cyl2 z_cyl2] = cylinder(sc_cyl2_radius*[1 1], sc_cyl2_mesh_points);
h_cyl2                 = surf(x_cyl2, y_cyl2, sc_cyl2_height*z_cyl2 + sc_cyl2_z_offset, ...
                              'FaceColor', sc_cyl1_colour, 'FaceAlpha', sc_cyl2_transparency, 'EdgeAlpha', sc_cyl2_transparency);


% -------------------------------------------------------------------------
% add payload dish to figure
% -------------------------------------------------------------------------
h_payloaddish = PartialSphere(dish_mesh_points, dish_base_offset, dish_radius, dish_phi, dish_colour, 'k', dish_transparency);
                          
% -------------------------------------------------------------------------
% define vertices of patch object for each solar array, each row gives [X Y
% Z] points of one vertex, and add solar arrays to figure 
% -------------------------------------------------------------------------
% loop through for each of three SA plates
pa_x_offset = sa_x_offset;
sa_spacer   = 0.05*sc_body_width;
h_SA        = [];
for i=1:3
    % assign panel offset
    pa_x_offset   = sa_x_offset + (i-1)*(sa_length+sa_spacer);
    
    % define vertices for each solar array, each row is [X Y Z] point of
    % one vertex
    SA_vertices   = [        0  +sa_width*cos(sa_angle)  -sa_width*sin(sa_angle); 
                             0  -sa_width*cos(sa_angle)  +sa_width*sin(sa_angle);  
                     sa_length  -sa_width*cos(sa_angle)  +sa_width*sin(sa_angle);   
                     sa_length  +sa_width*cos(sa_angle)  -sa_width*sin(sa_angle)];
    SA1_vertices  = [SA_vertices(:,1)+pa_x_offset            SA_vertices(:,2:3)];
    SA2_vertices  = [SA_vertices(:,1)-sa_length-pa_x_offset  SA_vertices(:,2:3)];

    % add to figure
    h_SA1         = patch(SA1_vertices(:,1), SA1_vertices(:,2),SA1_vertices(:,3), sa_colour);
    h_SA2         = patch(SA2_vertices(:,1), SA2_vertices(:,2),SA2_vertices(:,3), sa_colour);
    set(h_SA1, 'FaceAlpha', sa_transparency)
    set(h_SA2, 'FaceAlpha', sa_transparency)
    
    % store handles
    h_SA          = [h_SA h_SA1 h_SA2];
end


% -------------------------------------------------------------------------
% Add SA hinges
% -------------------------------------------------------------------------
SA_hinge   = [sc_body_width                      0                        0; 
              sa_x_offset  +sa_width*cos(sa_angle)  -sa_width*sin(sa_angle);
              sa_x_offset  -sa_width*cos(sa_angle)  +sa_width*sin(sa_angle);
              sc_body_width                      0                        0];
h_SAhinge1 = plot3(+SA_hinge(:,1), +SA_hinge(:,2), +SA_hinge(:,3), 'LineWidth', sa_hinge_linewidth, 'Color', sa_hinge_colour);
h_SAhinge2 = plot3(-SA_hinge(:,1), -SA_hinge(:,2), -SA_hinge(:,3), 'LineWidth', sa_hinge_linewidth, 'Color', sa_hinge_colour);


% -------------------------------------------------------------------------
% Return array of handles to 3D objects
% -------------------------------------------------------------------------
h_aeolus   = [h_body h_cyl1 h_cyl1edge h_tripod1 h_tripod2 h_tripod3 h_cyl2 h_payloaddish h_SA h_SAhinge1 h_SAhinge2];