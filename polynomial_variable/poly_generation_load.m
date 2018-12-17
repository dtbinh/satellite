function data = poly_generation_load(filename, num)
    fileID = fopen(filename);
    formattable   = '%f';
    formatfloat   = '| %f ';
    
    
    for i = 1:num-1
        formattable = strcat(formattable,formatfloat);
    end

    filedata = textscan(fileID,formattable);
  
    for i = 1:num
        data(:,i) = filedata{i};
    end 
    
    fclose(fileID);

end


