function h_spacecraft = Vis3DSpacecraft(scale)
% Generates 3D image of spacecraft & adds to current figure.

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

% solar arrays
sa_width             = sc_body_width*1;       % half width of solar arrays (short axis of array)
sa_length            = sc_body_width*5;       % length of each solar array panel (long axis of array)
sa_angle             = 0;                     % solar array roll angle
sa_colour            = 'b';                   % solar array colour
sa_transparency      = 0.5;                   % solar array transparency
sa_x_offset          = sc_body_width*(1 + 0.5);
sa_hinge_linewidth   = 3*scale;
sa_hinge_colour      = 0.6*[1 1 0];


% -------------------------------------------------------------------------
% define vertices of patch object for S/C body & add s/c body to figure
% -------------------------------------------------------------------------
body_vertices_x = (2*[0 1 1 0 0 0; 1 1 0 0 1 1; 1 1 0 0 1 1; 0 1 1 0 0 0] - 1)*sc_body_width;
body_vertices_y = (2*[0 0 1 1 0 0; 0 1 1 0 0 0; 0 1 1 0 1 1; 0 0 1 1 1 1] - 1)*sc_body_width;
body_vertices_z = (2*[0 0 0 0 0 1; 0 0 0 0 0 1; 1 1 1 1 0 1; 1 1 1 1 0 1] - 1)*sc_body_width;
h_body          = patch(body_vertices_x, body_vertices_y, body_vertices_z, sc_colour);
set(h_body, 'FaceAlpha', sc_transparency)


% -------------------------------------------------------------------------
% define vertices of patch object for each solar array, each row gives [X Y
% Z] points of one vertex, and add solar arrays to figure 
% -------------------------------------------------------------------------

% define vertices for each solar array, each row is [X Y Z] point of
% one vertex
SA_vertices   = [        0  +sa_width*cos(sa_angle)  -sa_width*sin(sa_angle); 
	                     0  -sa_width*cos(sa_angle)  +sa_width*sin(sa_angle);  
                 sa_length  -sa_width*cos(sa_angle)  +sa_width*sin(sa_angle);   
	             sa_length  +sa_width*cos(sa_angle)  -sa_width*sin(sa_angle)];
	
SA1_vertices  = [SA_vertices(:,1)+sa_x_offset            SA_vertices(:,2:3)];
SA2_vertices  = [SA_vertices(:,1)-sa_length-sa_x_offset  SA_vertices(:,2:3)];

% add to figure
h_SA1         = patch(SA1_vertices(:,1), SA1_vertices(:,2),SA1_vertices(:,3), sa_colour);
h_SA2         = patch(SA2_vertices(:,1), SA2_vertices(:,2),SA2_vertices(:,3), sa_colour);
set(h_SA1, 'FaceAlpha', sa_transparency)
set(h_SA2, 'FaceAlpha', sa_transparency)

% store handles
h_SA          = [h_SA1 h_SA2];
				 

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
h_spacecraft = [h_body h_SA h_SAhinge1 h_SAhinge2];