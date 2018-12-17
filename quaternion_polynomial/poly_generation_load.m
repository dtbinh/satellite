function data = poly_generation_load(filename, num)
    fileID = fopen(filename);
    i = 1;
    filedata = textscan(fileID,'%f | %f');
    
    data(:,1) = filedata{1};
    data(:,2) = filedata{2};
%     while ~feof(fileID)
%         for n = 1:1:num
%             data(i,n)  = textscan(fileID,'%f |',1)
%         end
%         i = i+1;
%     end
    
    
    
    fclose(fileID);

end


