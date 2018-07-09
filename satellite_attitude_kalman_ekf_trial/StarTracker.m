function q = StarTracker(u,qbn)

% % euler is XYZ Rotation
% phi   = u(1)/2; 
% theta = u(2)/2; 
% psi   = u(3)/2;
% 
% sph = sin(phi); sth = sin(theta); sps = sin(psi);
% cph = cos(phi); cth = cos(theta); cps = cos(psi);
% 
% % q is wxyz
% q = [cph*cth*cps+sph*sth*sps
%      sph*cth*cps-cph*sth*sps;
%      cph*sth*cps+sph*cth*sps;
%      cph*cth*sps-sph*sth*cps;
%      ];
q = eul2q(u,'xyz','xyzw');

[~,I] = max(abs(q));
q = q/norm(q)*sign(q(I,1)/qbn(I,1));

end