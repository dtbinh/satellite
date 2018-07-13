function plotfigure(t,x,name,scale)
if ~exist('scale','var')
    scale = Inf;
end

if ~exist('name','var')
    name = ' ';
end

n = length(x(:,1));

sig   = std(x(:,ceil(end/3):end),0,2);
central = mean(x(:,ceil(end/3):end),2);

ymax = central+scale*sig;
ymin = central-scale*sig;

figure
for i=1:n
    subplot(n,1,i)
    plot(t,x(i,:))
    grid on; hold on;
    axis([-inf inf ymin(i) ymax(i)])
    title(name)
end

end