function [] = Vis3DAnimateAttitude(t,q, nsample, st_title, figure_size, varargin)
% Vis3DAnimateAttitude. Creates animation of spacecraft attitude evolution
% given input quaternion ephemeris. Format:
%
%   [] = Vis3DAnimateAttitude(q_data, nsample, st_title, figure_size)
% Inputs:
%   q_data                [5xn] array of quaternion ephemeris, first row is time
%   nsample               [1] point subsampling integer (larger value, faster
%                         animation, but lower resolution
%   st_title              string label appended to figure window as title
%   figure_size           flag indicating whether figure is standard size (=1) or
%                         maximum supportable by screen (=2)



% Assign default figure title if none entered
if ~exist('st_title','var')
    st_title = '';
end

if ~exist('figure_size','var')
    figure_size = 1;
end

% Setup figure window with 3D spacecraft object & combine graphic object handles into single array
r_sphere                               = 5;
[h_mainaxes, h_textaxes, h_scaxes]     = Vis3DReferenceFrame(r_sphere, st_title, 'k');
h_spacecraft                           = Vis3DSpacecraft;
h_spacecraft                           = [h_spacecraft h_scaxes];

% Add locus if supplied as input
st_style                               = {'w','g+','y','r'};
for i=1:length(varargin)
    unit_locus = varargin{i};
    unit_locus = unit_locus./ repmat((unit_locus(1,:).^2 + unit_locus(2,:).^2 + unit_locus(3,:).^2).^0.5, 3, 1);
    plot3(r_sphere*unit_locus(1,:),r_sphere*unit_locus(2,:),r_sphere*unit_locus(3,:),st_style{i})
end


% Rotate spacecraft object to starting attitude (noting that default
% attitude after above steps is [0 0 0 1])
Vis3DRotateSpacecraft(h_spacecraft,q(1:4,1),[0 0 0]);

% Add time label to figure text box
axes(h_textaxes);
text(0.05,-0.3,'Time','Color',[1 1 1]);
h_time  = text(0.35,-0.3,'= N/A','Color',[1 1 1]);
axes(h_mainaxes);

% Set figure window size
if figure_size==2
    screensize = get(0,'ScreenSize');
    set(gcf,'Position',[0 0 screensize(3)*0.5 screensize(4)]);
end

% Subsample data
q       = q(:,1:nsample:end);
t       = t(:,1:nsample:end);
t_ephemeris  = t(1,:);
npts         = length(t_ephemeris);
dt           = max(t_ephemeris)/20;


% -------------------------------------------------------------------------
% Run animation
% -------------------------------------------------------------------------
next_update_time = 0;
for i=2:npts
    
    % calculate rotation axis & angle over last time step
    q_prev   = q(1:4,i-1);
    q_curr   = q(1:4,i);
    qrot     = qmult([-q_prev(1:3); q_prev(4)], q_curr);    % rotation quaternion from previous to current attitude    
    [v, phi] = q2axisangle(qrot);                           % rotation axis & angle over last time step
    dcm      = q2dcm([-q_prev(1:3); q_prev(4)]);  
    v        = dcm*v;          % transform body referenced rotation vector into reference frame referenced vector
    
    % rotate 3D object to animate attitude response
    if phi~=0
        rotate(h_spacecraft, v, phi*180/pi);
    end

    % update progress indicator in figure window
     if t_ephemeris(i)>=next_update_time
        axes(h_textaxes);
        set(h_time,'String',['= ',num2str(t_ephemeris(i),'%0.0f'),'s']);
        next_update_time = next_update_time + dt;
        axes(h_mainaxes);
    end

    % force redraw of figure window
    drawnow;
    
end
