function R = DCM(i,theta)
% Matrix for Coordinate Transformation (Verified) 
% [James Diebel] Representing Attitude: Euler Angles, Unit Quaternions, and Rotation
% [John_Crassidis] Optimal Estimation Dynamic Systems

R = zeros(3,3);
switch i
    case 1
        R = [      1         0           0     ;
                   0     cos(theta)  sin(theta);
                   0    -sin(theta)  cos(theta)];
    case 2
        R = [ cos(theta)     0      -sin(theta);
                  0          1           0     ;
              sin(theta)     0       cos(theta)];
        
    case 3
        R = [  cos(theta)  sin(theta)    0     ;
              -sin(theta)  cos(theta)    0     ;
                   0           0         1     ];
        
    otherwise
        
end

end