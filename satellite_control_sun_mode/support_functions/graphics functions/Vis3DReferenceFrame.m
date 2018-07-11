function [h_mainaxes, h_textaxes, h_scaxes] = Vis3DReferenceFrame(radius, st_title, fig_colour)
% Vis3DReferenceFrame. Creates figure window including:
%   - info text box
%   - spacecraft & reference frame axes
%   - celestial sphere
% Can be used in conjunction with Vis3DSpacecraft to generate figure
% which includes spacecraft representation.
%
% Format:
%   [h_mainaxes, h_textaxes, h_scaxes] = Vis3DReferenceFrame(radius, st_title, st_subtitle, fig_colour)
% Inputs:
%   radius          Real        radius of transparent sphere added to figure
%   st_title        String      figure title
%   fig_colour      Char        character indicating background colour of
%                               figure - either 'k' (black) or 'w' (white)
% Outputs:
%   h_mainaxes      Real        handle to main figure axes
%   h_textaxes      Real        handle to text box axes (in which figure
%                               title is displayed)
%   h_scaxes        Real        [1x3] handles to s/c axes objects


% define axis labels
sc_frame_axes_labels  = {'X_{sc}' 'Y_{sc}' 'Z_{sc}'};
ref_frame_axes_labels = {'X_{ref}' 'Y_{ref}' 'Z_{ref}'};

% determine graph colour - either black or white background is supported
if strcmp(fig_colour,'k')
    axis_colour = [1 1 1];
    bkg_colour  = 'k';
else
    axis_colour = 0*[1 1 1];
    bkg_colour  = 'w';
end
    
% create basic figure window
[h_figure, h_mainaxes] = Vis3DSetupFigure(st_title, bkg_colour);
h_textaxes             = Vis3DSetupFigureTextBox(st_title, '', axis_colour);
h_sphere               = Vis3DSphere(radius, h_mainaxes, 0);
h_scaxes               = Vis3DAddFrameAxes([0 0 0]', sc_frame_axes_labels, axis_colour, radius);
h_refaxes              = Vis3DAddFrameAxes([0 0 0]', ref_frame_axes_labels, axis_colour*0.8, radius);
hold on

return