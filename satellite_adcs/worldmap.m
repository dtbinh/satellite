%% WORLD MAP PLOT

ax = worldmap([35 45],[20 35]);
screensize = get(0,'ScreenSize');
% load coastlines
% plotm(coastlat,coastlon)

land = shaperead('landareas', 'UseGeoCoords', true);
geoshow(ax, land, 'FaceColor', [0.5 0.7 0.5])

plotm([latSSP1 latSSP2]/pi*180,[lonSSP1 lonSSP2]/pi*180,'kx-')
textm(latSSP1/pi*180,lonSSP1/pi*180,'SSP1')
textm(latSSP2/pi*180,lonSSP2/pi*180,'SSP2')


plotm([latWTarget1 latWTarget2]/pi*180,[lonWTarget1 lonWTarget2]/pi*180,'bo-')
textm(latWTarget1/pi*180,lonWTarget1/pi*180,'WT1','Color','blue')
textm(latWTarget2/pi*180,lonWTarget2/pi*180,'WT2','Color','blue')

plotm([latETarget1 latETarget2]/pi*180,[lonETarget1 lonETarget2]/pi*180,'ro-')
textm(latETarget1/pi*180,lonETarget1/pi*180,'ET1','Color','red')
textm(latETarget2/pi*180,lonETarget2/pi*180,'ET2','Color','red')
