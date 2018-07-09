itermax      = 3;
phiN_array   = [180 2 0.05];
thetaN_array = [90 2 0.05];
phimax = pi;
thetamax = 0;
for iter = 1:1:itermax
Amax = 0;
N = 10^(2*iter);
phimid   = phimax/pi*180;
thetamid = thetamax/pi*180;
phiN = phiN_array(iter);
thetaN = thetaN_array(iter);
i = 1;
for phi=(phimid-phiN)/180*pi:pi/N:(phimid+phiN)/180*pi
        j = 1;
    for theta = (thetamid-thetaN)/180*pi:pi/N:(thetamid+thetaN)/180*pi
          
        R_S_I = DCM(2,theta)*DCM(3,phi); % Rotation From Inertial To Sun Vector
        R_I_S = R_S_I';                  % Rotation From Sun Vector To Inertial
        V_S = R_I_S*[1;0;0];             % Sun vector in Inertial Frame
 
        Area = 0; % Reset Area
        
        for k=1:1:6
            if (norm(A_SP(:,k))~=0)
                cosalpha = dot(V_S,A_SP(:,k))/norm(V_S)/norm(A_SP(:,k));
            else
                cosalpha = 0;
            end
            
           
           Area = Area + (cosalpha>0)*cosalpha*norm(A_SP(:,k)); 
        end
         if Area>Amax
             Amax = Area;
             phimax = phi;
             thetamax = theta;
             vmax = V_S;
         end
        j = j+1;
    end
     i = i+1; 
end   

%% OUTPUT
fprintf("Iteration Number = %d\n", iter);
fprintf("Solar Panel Array = [%d %d %d %d %d %d]\n",SP_Area);
fprintf("Angle Resolution = %.8f\n",pi/N);
fprintf("Maximum Effective Area = %.8f\n", Amax);
fprintf("Optimum Sun Vector in Body Frame = [%.6f %.6f %.6f]\n", vmax);
fprintf("phi = %.6f  theta = %.6f (euler coordinates)\n", phimax/pi*180, thetamax/pi*180);
fprintf("Optimum Power Generation Loss = %.2f\n\n", Amax/Area_eff*100);
end

%% BST Formulation
% Ax = 2;
% Ay = -6;
% Az = -2;
% 
% theta1 = 107.5/180*pi;
% phi1   = 71.6/180*pi;
% 
% V_S = [0.3015;-0.9045;-0.3015];             % Sun vector in Inertial Frame
%  
%         Area = 0; % Reset Area
%         
%         for k=1:1:6
%             if (norm(A_SP(:,k))~=0)
%                 cosalpha = dot(V_S,A_SP(:,k))/norm(V_S)/norm(A_SP(:,k));
%             else
%                 cosalpha = 0;
%             end
%             
%            
%            Area = Area + (cosalpha>0)*cosalpha*norm(A_SP(:,k)); 
%         end
% Area
% fprintf("Maximum Effective Area = %.8f\n", Aeff(x,y));
% Eff_A = Ax*sin(theta)*cos(phi)+Ay*sin(theta)*sin(phi)+Az*cos(theta)
