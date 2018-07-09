function euler = Quat2Euler(q,type)

switch type
    case 'xyzw'
        quat(1) = q(1); % x Component 
        quat(2) = q(2); % y Component 
        quat(3) = q(3); % z Component 
        quat(4) = q(4); % w Component 
    case 'wxyz'
        quat(1) = q(2); % x Component 
        quat(2) = q(3); % y Component 
        quat(3) = q(4); % z Component 
        quat(4) = q(1); % w Component 
    otherwise
end

euler = zeros(3,1);

a = 2*(quat(4)*quat(2)-quat(3)*quat(1));

if a > 1
    a = 1;
elseif a < -1
    a = -1;
end
    euler(1)= atan2( 2*(quat(1)*quat(4)+quat(2)*quat(3)), 1-2*(quat(1)*quat(1)+quat(2)*quat(2)));
    euler(2)= asin(a);
    euler(3)= atan2( 2*(quat(3)*quat(4)+quat(1)*quat(2)), 1-2*(quat(2)*quat(2)+quat(3)*quat(3)));
return


end