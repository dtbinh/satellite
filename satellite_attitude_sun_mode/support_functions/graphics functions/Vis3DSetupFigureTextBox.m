function h_textaxes = Vis3DSetupFigureTextBox(st_title, st_subtitle, text_colour)
% Vis3DSetupFigureTextBox

h_originalaxes  = gca;
h_textaxes      = axes('position',[0.05 0.95 0.2 0.2]);  
set(h_textaxes,'Visible','off');
text(0,0,st_title,'Color',text_colour, 'FontSize', 14)
text(0,-0.15,st_subtitle,'Color',text_colour, 'FontSize', 10)
axes(h_originalaxes);
