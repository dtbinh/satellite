[data,txt] = xlsread('igrf11coeffs.xls'); 

year = 2011;
day  = 53;

%% RUN THROUGH THE DATA
O = 1;       % Because n,m is starts from 0 to 13, there is a need to add 1

for i = 1:length(data)
    col1 = data(i,1);
    col2 = data(i,2);
    col3 = data(i,3);
    col4 = data(i,4);
    
    if strcmp(txt(i),'g')
          g_data(O+col1,O+col2) = col3;
        g_dataSV(O+col1,O+col2) = col4;
    elseif strcmp(txt(i),'h')
          h_data(O+col1,O+col2) = col3;
        h_dataSV(O+col1,O+col2) = col4;
            
    end
end


g_data = g_data+((year-2010)+day/365)*g_dataSV ;
h_data = h_data+((year-2010)+day/365)*h_dataSV;

save IGRF11_data g_data h_data

%% GENERATE C HEADER FILE
filename = 'IGRFcoeffs.h';

load('IGRF11_data');
fid = fopen(filename,'w');

fprintf(fid,'static const double h_data[14*14] = {+\n');

L = size(h_data,1);

O = 1;

for i = 0:L-1
    for j = 0:L-1
        fprintf(fid,'%.3f,',h_data(i+O,j+O));
        if mod(i*L+j+1,10)== 0
            fprintf(fid,'+\n');
        end
    end
end

fseek(fid,-1,'eof');
fprintf(fid,'');

fprintf(fid,'};\n\n');

fprintf(fid,'static const double g_data[14*14]={+\n');

for i=0:L-1
    for j = 0:L-1
        fprintf(fid,'%.3f,',g_data(i+O,j+O));
        if mod(i*L+j+1,10)==0
            fprintf(fid,'+\n');
        end
    end
end

fseek(fid,-1,'eof');
fprintf(fid,'');
fprintf(fid,'};\n\n');
fprintf(fid,'\n#endif/*_IGRFcoeffs_*/\n');

fclose(fid);









    