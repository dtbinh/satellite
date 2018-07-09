function [Ad_b, Bd_b, Cd_b]= linear_eq(Ixx,Iyy,Izz,Ia,DCM_w_b,dt)

    
D11 = DCM_w_b(1,1);
D12 = DCM_w_b(1,2); 
D13 = DCM_w_b(1,3); 
D21 = DCM_w_b(2,1); 
D22 = DCM_w_b(2,2); 
D23 = DCM_w_b(2,3); 
D31 = DCM_w_b(3,1); 
D32 = DCM_w_b(3,2); 
D33 = DCM_w_b(3,3); 

%% LINEARIZED AIR BEARING STATE SPACE
A_b = [0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0];
    
B41 = -Ia*D11/Ixx;
B42 = -Ia*D12/Ixx;
B43 = -Ia*D13/Ixx;
B51 = -Ia*D21/Iyy;
B52 = -Ia*D22/Iyy;
B53 = -Ia*D23/Iyy;
B61 = -Ia*D31/Izz;
B62 = -Ia*D32/Izz;
B63 = -Ia*D33/Izz;
    
    
B_b = [0   0   0   ;
        0   0   0   ;
        0   0   0   ;
        B41 B42 B43 ;
        B51 B52 B53 ;
        B61 B62 B63 ];
    
C_b = [1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1];
    
D_b = [ 0   0   0   ;
        0   0   0   ;
        0   0   0   ;
        0   0   0   ;
        0   0   0   ;
        0   0   0   ];
sys = ss(A_b, B_b, C_b, D_b);
sysd = c2d(sys,dt);
%% DISCRETE AIR BEARING STATE SPACE
I    = diag([1 1 1 1 1 1]);
Ad_b = expm(A_b*dt);
%Bd_b = A_b^-1*(Ad_b-I)*B_b;

fun = @(t) expm(A_b*t);

Bd_b = integral(fun,0,dt,'ArrayValued',true)*B_b;
Cd_b = C_b;
end