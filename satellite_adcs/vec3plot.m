function vec3plot(tout,vec3,ttext,xtext,ytext)
if ~exist('ttext','var')
   ttext = ' '; 
end

if ~exist('xtext','var')
   xtext = 'time [s]'; 
end

if ~exist('ytext','var')
   ytext = ' '; 
end

figure;
subplot(3,1,1)
plot(tout,vec3(1,:))
hold on;grid on;
title(ttext);
ylabel(ytext);
xlabel(xtext);

subplot(3,1,2)
plot(tout,vec3(2,:))
hold on;grid on;
ylabel(ytext);
xlabel(xtext);

subplot(3,1,3)
plot(tout,vec3(3,:))
hold on;grid on;
ylabel(ytext);
xlabel(xtext);

end