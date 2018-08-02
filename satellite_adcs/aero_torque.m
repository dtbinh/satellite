function y = aero_torque(R,A,V,w,rho,type)
% This function calculates the torque due to aerodynamic disturbances based
% on either hughes theory or wertz theory. Input variables are:
% R - [m]   Matrix of Position Vectors of each dA
% A - [m^2] Matrix of Area Vector of each dA
% V - [m/s] Velocity Vector of Spacecraft in Body Frame
% w - [rad/s] Angular Velocity of Spacecraft in Body Frame
% rho - [kg/m^3] Density of Atmosphere


Vo  = V/norm(V);    % [] Unit Vector of Velocity
Vm  = norm(V);      % [] Velocity Magnitude

switch type
    case 'hughes'
    %% HUGHES SPACECRAFT ATTUTDE DYNAMICS (PAGE 252)

    for i=1:1:length(R(1,:))
        N = A(:,i)/norm(A(:,i));
        cosalpha(:,i) = dot(N,V+cross(w,R(:,i)))/norm(N)/norm(V+cross(w,R(:,i)));
%       cosalpha(:,i) = dot(N,V)/norm(N)/norm(V);

        term1(:,i) = (cosalpha(:,i)>0)*cosalpha(:,i)*(cross(Vo,R(:,i)))*norm(A(:,i)); % Effect of Instantaneous Center of Pressure
        term2(:,i) = (cosalpha(:,i)>0)*((norm(R(:,i))^2)*eye(3)-R(:,i)*R(:,i)')*norm(A(:,i))*cosalpha(:,i)*w; % Effect of Symmetry of External Surface Areas
        term3(:,i) = (cosalpha(:,i)>0)*Smtrx(Vo)*(R(:,i)*N'*(Smtrx(R(:,i))))*norm(A(:,i))*w; % Effect of the Angle of Surface Area Vector with respect to the Area Location Vector
    end
    y = rho*Vm^2*(sum(term1,2))...
        - rho*Vm*(sum(term2,2))...
        - rho*Vm*(sum(term3,2));

        
    case 'wertz'
    %% WERTZ SPACECRAFT ATTITUDE DETERMINATION AND CONTROL (PAGE 574)

    for i=1:1:length(R(1,:))
      N = A(:,i)/norm(A(:,i));

      term1(:,i) = (dot(N,Vo)>0)*dot(N,Vo)*(cross(Vo,R(:,i)))*norm(A(:,i));
      term2(:,i) = (dot(N,Vo)>0)*dot(N,cross(w,R(:,i)))*cross(Vo,R(:,i))*norm(A(:,i));
      term3(:,i) = (dot(N,Vo)>0)*dot(N,Vo)*(cross(cross(w,R(:,i)),R(:,i)))*norm(A(:,i));
    end


    y = rho*Vm^2*(sum(term1,2))...
        + rho*Vm*(sum(term2,2))...
        + rho*Vm*(sum(term3,2));

        
    otherwise
        
end








end