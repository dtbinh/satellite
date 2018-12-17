fileID = fopen(file);
wdata    = textscan(fileID,'%19c %f %f %f | %4c %f %11c %21c');
fclose(fileID);

% results.Name = wdata{1};
% utc_data  = wdata{2};
% utc_time  = wdata{3};
% dt   = wdata{4};
% q0   = wdata{5};
% q1   = wdata{6};
% q2   = wdata{7};
% q3   = wdata{8};
% ra   = wdata{9};
% de   = wdata{10};
% az   = wdata{11};
