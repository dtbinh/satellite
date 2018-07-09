function [h_figure, h_axes] = Vis3DSetupFigure(st_title, back_colour, fig_size)
% Vis3DSetupFigure

% open figure window & set standard options
h_figure = figure;
set(h_figure,'Color',back_colour,'UserData',1,'numbertitle','off','name',[num2str(get(gcf,'Number')),'. ',st_title]);

% set figure size if specified in input argument
if exist('fig_size')
    if strcmpi(fig_size,'max')
        screensize = get(0,'ScreenSize');
        set(h_figure,'Position',[50 50 screensize(3)-100 screensize(4)-250]);
    else
        set(h_figure,'Position',fig_size);
    end
end
        

% setup camera options
cameratoolbar('SetMode','orbit');
set(gcf,'KeyPressFcn',@animation_keypress_callback);

% get handle to current axes
h_axes = gca;


% =========================================================================
% Support functions
% =========================================================================
function animation_keypress_callback(h_figure,eventobj)

if(strcmp(get(h_figure,'CurrentCharacter'),' '))
    fig_flag = get(h_figure,'UserData');
    fig_flag = ~fig_flag;
    set(h_figure,'UserData',fig_flag);
end
