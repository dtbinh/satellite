function plot_bode(sys, w, t_delay, st_style)
% Plot Bode response of system "sys" with time delay "t_delay".

% note that Matlab bode function returns:
%   mag       = magnitude (raw, not in decibels)
%   phase     = phase angle in degrees
[mag phase]   = bode(sys,w);
mag           = squeeze(mag);
phase         = squeeze(phase); 

% modify phase to account for time delay (remembering units of phase are [deg])
phase         = phase - t_delay*w*180/pi;

% generate figure
subplot(2,1,1)
semilogx(w, 20*log10(mag),st_style), hold on, zoom on, grid on
ylabel('gain [dB] (=20log_{10}(|G(\omegaj)|)')

subplot(2,1,2)
semilogx(w, phase,st_style), hold on, zoom on, grid on
ylabel('phase [deg]')
xlabel('frequency \omega [deg/sec]')

% gain margin calculation
n = length(phase);
i = min(find(phase(1:n-1)>-180&phase(2:n)<-180));
if ~isempty(i)
    w_gm = w(i);
    gm   = -20*log10(mag(i));
    subplot(2,1,1)
    ax = axis;
    semilogx(w_gm*[1 1], ax(3:4),'k--')
    subplot(2,1,2)
    ax = axis;
    semilogx(w_gm*[1 1], ax(3:4),'k--')
    fprintf('    gain margin  = %0.2fdB (at %0.4fr/s)\n',gm,w_gm)
else
    fprintf('    gain margin  = infdB\n')
end

% phase margin calculation
i = min(find(mag(1:n-1)>1&mag(2:n)<1));
if ~isempty(i)
    w_pm = w(i);
    pm   = phase(i)+180;
    subplot(2,1,1)
    ax = axis;
    semilogx(w_pm*[1 1], ax(3:4),'k--')
    subplot(2,1,2)
    ax = axis;
    semilogx(w_pm*[1 1], ax(3:4),'k--')
    fprintf('    phase margin = %0.2fdeg (at %0.4fr/s)\n',pm,w_pm)
else
    fprintf('    phase margin = infdB\n')
end

return
