function Q = qmatrix(q,type)

switch type
    case 'wxyz'
        
        Q = [q(1) -q(2) -q(3) -q(4);
             q(2)  q(1)  q(4) -q(3);
             q(3) -q(4)  q(1)  q(2);
             q(4)  q(3) -q(2)  q(1)];
    case 'xyzw'
        Q = [ q(4)  q(3) -q(2)  q(1);
             -q(3)  q(4)  q(1)  q(2);
              q(2) -q(1)  q(4)  q(3);
             -q(1) -q(2) -q(3)  q(4)];
end

end