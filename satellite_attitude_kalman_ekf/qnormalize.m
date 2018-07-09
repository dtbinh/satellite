
function q = qnormalize(q)

qnorm = q'*q;

while (qnorm) > 1
    if qnorm > 1 + 1e-16
        q = ((3 + qnorm)/(1 + 3*qnorm))*q; % rescale quaternion to (err^3)/32     
    else
        q = q/sqrt(qnorm);                 % renormalize quaternion      
    end
        qnorm = q'*q;

end
return