function tau = aero_torque(R,A,V,w,rho,type)
% This function calculates the torque due to aerodynamic disturbances based
% on either hughes theory or wertz theory. Input variables are:
% R - [m]   Matrix of Position Vectors of each dA
% A - [m^2] Matrix of Area Vector of each dA
% V - [m/s] Velocity Vector of Spacecraft in Body Frame
% w - [rad/s] Angular Velocity of Spacecraft in Body Frame
% rho - [kg/m^3] Density of Atmosphere

if ~exist('type','var')
    type = 'wertz';
end


Vo  = V/norm(V);    % [] Unit Vector of Velocity
Vm  = norm(V);      % [] Velocity Magnitude

switch type
    case 'hughes' 
    %% HUGHES SPACECRAFT ATTUTDE DYNAMICS (PAGE 252)
        for i=1:1:length(R(1,:))
            N = A(:,i)/norm(A(:,i));
            Vr = -V-cross(w,R(:,i)); % Velocity of relative air in body frame
            Vrn = Vr/norm(Vr);       % Velocity of relative air in body frame

            % Heaviside Function
            cosalpha(:,i) = dot(N,V+cross(w,R(:,i)))/norm(N)/norm(V+cross(w,R(:,i)));

            
            I_term(:,:,i) = (cosalpha(:,i)>0)*(norm(R(:,i))^2)*eye(3)-R(:,i)*R(:,i)'*cosalpha(:,i)*norm(A(:,i));
            J_term(:,:,i) = (cosalpha(:,i)>0)*R(:,i)*A(:,i)'*(Smtrx(R(:,i)));
            Cp_term(:,i)  = (cosalpha(:,i)>0)*cosalpha(:,i)*R(:,i)*norm(A(:,i));

            % Summing up all the forces on each dA and calculating each dg
            f_term(:,i) = (cosalpha(:,i)>0)*rho*norm(Vr)^2*cosalpha(:,i)*norm(A(:,i))*Vrn;
            g_term(:,i) = cross(R(:,i),f_term(:,i));
        end

        I = sum(I_term,3); % Aerodynamic Moment of Inertia about Principle Axes
        J = sum(J_term,3); % Aerodynamic Moment of Inertia about Principle Axes
        Ac = sum(Cp_term,2);
        Vo = -Vo;

        Gc = rho*Vm*(Vm*Smtrx(Ac)*Vo-(I+Smtrx(Vo)*J)*w); 
        G = sum(g_term,2);
        
        tau = G;
    
        
    
    case 'wertz' 
    %% WERTZ SPACECRAFT ATTITUDE DETERMINATION AND CONTROL (PAGE 574)
        for i=1:1:length(R(1,:))

          N = A(:,i)/norm(A(:,i)); % unit vector normal to Surface Area

          term1(:,i) = (dot(N,Vo)>0)*dot(N,Vo)*(cross(Vo,R(:,i)))*norm(A(:,i));
          term2(:,i) = (dot(N,Vo)>0)*dot(N,cross(w,R(:,i)))*cross(Vo,R(:,i))*norm(A(:,i));
          term3(:,i) = (dot(N,Vo)>0)*dot(N,Vo)*(cross(cross(w,R(:,i)),R(:,i)))*norm(A(:,i));
        end

        tau = rho*Vm^2*(sum(term1,2))+ rho*Vm*(sum(term2,2))+ rho*Vm*(sum(term3,2));

end


end