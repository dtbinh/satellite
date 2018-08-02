function q = eul2q(eul,etype,qtype)

phi   = eul(1); 
theta = eul(2);  
psi   = eul(3); 

switch etype
    case 'zyx'
        
        switch qtype
            case 'wxyz'
 
                qx = [cos(phi/2);sin(phi/2);0;0];
                qy = [cos(theta/2);0;sin(theta/2);0];
                qz = [cos(psi/2);0;0;sin(psi/2)];

                q = qmatrix(qz,qtype)*qmatrix(qy,qtype)*qx;
                
             case 'xyzw'
                qx = [sin(phi/2);0;0;cos(phi/2)];
                qy = [0;sin(theta/2);0;cos(theta/2)];
                qz = [0;0;sin(psi/2);cos(psi/2)];

                q = qmatrix(qz,qtype)*qmatrix(qy,qtype)*qx;
        end
        
     case 'xyz'
         switch qtype
            case 'wxyz'
 
                qx = [cos(phi/2);sin(phi/2);0;0];
                qy = [cos(theta/2);0;sin(theta/2);0];
                qz = [cos(psi/2);0;0;sin(psi/2)];

                q = qmatrix(qx,qtype)*qmatrix(qy,qtype)*qz;
                
             case 'xyzw'
                 
                qx = [sin(phi/2);0;0;cos(phi/2)];
                qy = [0;sin(theta/2);0;cos(theta/2)];
                qz = [0;0;sin(psi/2);cos(psi/2)];

                q = qmatrix(qx,qtype)*qmatrix(qy,qtype)*qz;
        end
        
end



end