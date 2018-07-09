function [sys,x0,str,ts] = sfun_simprogress(t,x,u,flag,update_interval)
% Outputs the current simulation time to MATLAB at regular intervals to inform the user on progress.

switch flag,

  % Initialization
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(update_interval);

  % Update
  case 3,
    sys=mdlOutputs(t,x,u);

  % Unused
  case {1,2,9}
    sys=[];

  % Unexpected flags
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end


%=============================================================================
% mdlInitializeSizes
%=============================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(update_interval)

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 0;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];
str = [];

% initialize the array of sample times
ts  = [update_interval 0];

% end mdlInitializeSizes

%=============================================================================
% mdlOutputs
%=============================================================================
function sys=mdlOutputs(t,x,u)

if (t==0)
    disp('----------------------------------------------------------------');
end
disp(['time = ',num2str(t,'%0.0f'),'s'])
sys = [];

% end mdlOutputs

