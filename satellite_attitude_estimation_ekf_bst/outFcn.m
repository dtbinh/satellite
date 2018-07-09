function status = outFcn(t,~,flag)


global tmpVAR VAR;
persistent ite; % ite is only global to this function

switch flag
    case 'init' 
        % Setup for plots or dialog boxes
        ite    = 1;
        VAR.moment = zeros(10000,3);
        VAR.torque = zeros(10000,3);
        VAR.J = zeros(10000,1);
        VAR.t = zeros(10000,1);
        
        status = 0;
    case 'iter'
        % Make updates to plots or dialog boxes as needed
    case 'interrupt'
        % Check conditions to see whether optimization should quit        
    case 'done' 
        % Cleanup of plots, dialog boxes, or final plot
        VAR.moment(ite+1:end,:) = [];
        VAR.torque(ite+1:end,:) = [];
        VAR.J(ite+1:end,:) = [];
        VAR.t(ite+1:end,:) = [];
        
        status = 0;
    otherwise 
        ite = ite +1;
        VAR.moment(ite,:) = tmpVAR.moment';
        VAR.torque(ite,:) = tmpVAR.torque';
        VAR.W(ite,:) = tmpVAR.W;
        VAR.t(ite,1) = t(end);
        
        status = 0;
        
end

end
        