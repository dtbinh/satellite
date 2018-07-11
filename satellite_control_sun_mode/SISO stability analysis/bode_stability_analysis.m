% Bode stability analysis of simplified SISO control loops for ESM
% (Emergency Sun Mode) described in P6 notes.

% -------------------------------------------------------------------------
% preliminaries
% -------------------------------------------------------------------------
clear variables
close all
clc

% define frequency range to construct Bode response over
w      = logspace(-3,1,1001)';

% -------------------------------------------------------------------------
% attitude controller stability - Y axis example from P6 notes
% -------------------------------------------------------------------------
disp('Attitude Controller:')
I      = 30000;
Krcs   = 1;
Kp     = 375;
Kd     = 15000;
tdelay = 3;

sys    = tf([0 Kd*Krcs Kp*Krcs],[I 0 0]);
% sys    = tf([0 0 Kp*Krcs],[I Kd*Krcs 0]);

figure
disp('  system without time delay')
plot_bode(sys, w, 0, 'b')               % system with no time delay
disp('  system with time delay')
plot_bode(sys, w, tdelay, 'r')          % system with time delay
subplot(2,1,1)
title('Bode response of attitude controller with and without delays')
subplot(2,1,2)
ax = axis;
axis([ax(1:2),-600, 0])


% -------------------------------------------------------------------------
% rate controller stability - Z axis example from P6 notes
% -------------------------------------------------------------------------
disp('Rate Controller:')
I      = 30000;
Kd     = 1150;
tdelay = 3;

sys    = tf([0 Kd*Krcs],[I 0]);

figure
disp('  system without time delay')
plot_bode(sys, w, 0, 'b')               % system with no time delay
disp('  system with time delay')
plot_bode(sys, w, tdelay, 'r')          % system with time delay

subplot(2,1,1)
title('Bode response of rate controller with and without delays')
