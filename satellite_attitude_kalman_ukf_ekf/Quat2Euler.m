function euler = Quat2Euler( quat )
% xyzw
euler=zeros(3,1);
a = 2*(quat(4)*quat(2)-quat(3)*quat(1));
if a > 1
a = 1;
elseif a < -1
a = -1;
end

euler(1)=atan2( 2*(quat(1)*quat(4)+quat(2)*quat(3)), 1-2*(quat(1)*quat(1)+quat(2)*quat(2)));
euler(2)=asin(a);
euler(3)=atan2( 2*(quat(3)*quat(4)+quat(1)*quat(2)), 1-2*(quat(2)*quat(2)+quat(3)*quat(3)));
return