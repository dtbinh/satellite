function [P, dP, S] = Pfunk(n,m,theta)
O = 1;

P  =  ones(n+O, m+O);
dP = zeros(n+O, m+O);
S  =  ones(n+O, m+O);

i = n;
j = m;

% Calculating S matrix
for n = 0:1:i
    for m = 0:1:j
        if n > 0
            if m == 0
                S(n+O, m+O) = S(n+O-1, m+O)*(2*n-1)/(n-m);
            else
                if m == 1
                    deltaFun = 1;
                else
                    deltaFun = 0;
                end
            
                S(n+O, m+O) = S(n+O, m+O-1)*sqrt(((n-m+1)*(deltaFun+1))/(n+m));
            end
        end
    end
end

% Calculates the Legendre Polynominal P and its partial 

for n = 0:1:i
    for m = 0:1:j
        if n ==1
            dP(n+O,m+O) = cos(theta)*dP(n-1+O,m+O)-sin(theta)*P(n-1+O,m+O);
            P(n+O,m+O)  = cos(theta)*P(n-1+O,m+O);
        elseif n>1
            K = ((n-1)^2-m^2)/((2*n-1)*(2*n-3));
            dP(n+O,m+O) = cos(theta)*dP(n-1+O,m+O)-sin(theta)*P(n-1+O,m+O)-K*dP(n-2+O,m+O);
            P(n+O,m+O)  = cos(theta)*P(n-1+O,m+O) - K*P(n-2+O,m+O);
        end
        
        if n == m && n>0
             P(n+O,m+O) = sin(theta)*P(n-1+O,n-1+O);
            dP(n+O,m+O) = sin(theta)*dP(n-1+O,n-1+O)+cos(theta)*P(n-1+O,n-1+O);
        end
    end
end
     
end