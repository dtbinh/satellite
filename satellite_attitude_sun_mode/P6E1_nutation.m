function T = P5E1_nutation(input)
t = input(1);
q = input(2:5);

DCM = q2dcm(q);
if (t>=200&&t<=205)

	T_b = DCM*[0 ; 34.907 ;0];
else if (t>=581.84&&t<=586.84)

        T_b = DCM*[- 34.907 ;0 ;0];
    else
        T_b= [0 ;0 ;0];
    end
end

T=T_b;
return