%  This function save figure as pdf.
%               
%  inputs                         
%    fig        - figure to be saved
%    filename   - name of the file to save as, excluding .pdf
%    size       - A3,A4,A5
%
%  outputs    :
%
%
%  author     : rusty          - initiate     05 dec 2018               
% ------------------------------------------------------------------------
function savepdf(fig,filename,size)

if ~exist('size','var')
    size = 'A4'; 
end


switch size
    case 'A4'
        set(fig,'PaperSize',[21 29.7]);
        set(fig,'PaperPosition',[0.0 0.0 21 29.7]);
        saveas(fig,filename,'pdf');
    otherwise
        fprintf('Size is not correct\n');
        

end