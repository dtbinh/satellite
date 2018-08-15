function [dcm,r1,r2,r3] = triad2dcm(vec1, vec2)

r1 = vnorm(vec1);
r3 = vnorm(cross(r1,vec2));
r2 = cross(r3,r1);

dcm = [r1 r2 r3];
end