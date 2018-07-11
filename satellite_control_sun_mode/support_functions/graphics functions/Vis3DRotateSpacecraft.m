function [] = Vis3DRotateSpacecraft(h_spacecraft,q,r)
% Vis3DRotateSpacecraft. Rotate spacecraft object by quaternion 'q', with
% the origin of rotation r. The quaternion q is relative to the plot frame,
% rather than the object frame.

q        = reshape(q,4,1);
qrot     = sign(q(4))*q;
[v, phi] = q2axisangle(qrot);
if phi~=0
    rotate(h_spacecraft, v, phi*180/pi, r);
end

return