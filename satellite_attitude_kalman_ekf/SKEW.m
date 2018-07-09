%#eml
function sk = SKEW( vec )

%% PRE-CHECK
if ~(( (size(vec,1) == 3) && (size(vec,2) == 1) ) || ( (size(vec,1) ==1) && (size(vec,2) == 3) ))
    disp('not a vector');
    return
end
%% FUNCTION MAIN
sk = [   0    -vec(3)  vec(2);
       vec(3)    0    -vec(1);
      -vec(2) vec(1)     0 ];
return