fileID = fopen(file);
qheader1 = textscan(fileID,'%4c | %31c| %22c | %24c | %18c |',1);
qheader2 = textscan(fileID,'%s | %s | %9c | %4c | %4c | %4c | %4c |%8c |%8c |%8c |%5c |%5c |%5c |%5c |%5c |%5c |%9c |%5c |%5c |%5c |%5c |%5c |%5c |%10c |%4c |%4c |%4c |%4c |%5c |%5c |%5c |%9c |%9c |%9c |%c|%c|%11c|%9c|%9c|%11c|',1);
qdata    = textscan(fileID,'%d | %s %s |%d | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %d | %d | %s | %f | %f | %f | ');
fclose(fileID);

results.Name = qdata{1};
utc_data  = qdata{2};
utc_time  = qdata{3};
dt   = qdata{4};
q0   = qdata{5};
q1   = qdata{6};
q2   = qdata{7};
q3   = qdata{8};
ra   = qdata{9};
de   = qdata{10};
az   = qdata{11};

hb0   = qdata{12};
hb1   = qdata{13};
hb2   = qdata{14};
hi0   = qdata{15};
hi1   = qdata{16};
hi2   = qdata{17};

habs  = qdata{18};
wb0  = qdata{19};
wb1  = qdata{20};
wb2  = qdata{21};
wi0  = qdata{22};
wi1  = qdata{23};
wi2  = qdata{24};
wabs  = qdata{25};

qtgt0  = qdata{26};
qtgt1  = qdata{27};
qtgt2  = qdata{28};
qtgt3  = qdata{29};

wtgtb0  = qdata{30};
wtgtb1  = qdata{31};
wtgtb2  = qdata{32};
wtgti0  = qdata{33};
wtgti1  = qdata{34};
wtgti2  = qdata{35};

V = qdata{36};
T = qdata{37};

NADIR = qdata{37};
STact_sun = qdata{38};
STact_earth = qdata{39};
