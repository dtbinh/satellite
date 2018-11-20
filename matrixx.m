close all
clear all
clc
format 
% Mat3 = [
%  0.000000162397384429 -0.000000000000002282 -0.000000000000000487 ;
% -0.000000000000003315 0.000000162396432302 0.000000000000006879 ;
% -0.000000000000001253 0.000000000000000748 0.000000162397384429];
Mat3 = [
 2 -1 0.0;
 -1 2 -1;
 0.0 -1 2
]


%%
fprintf('===================Method 2==============');

Mat1 = [Mat3 eye(3)]
MatS = Mat1(1:3,1:3);

Mat = Mat1;
for i=1:1:6
	Mat1(1,i) = Mat(1,i)-Mat(1,3)/Mat(3,3)*Mat(3,i);
end

Mat = Mat1;
for i=1:1:6   
    Mat1(2,i) = Mat(2,i)-Mat(2,3)/Mat(3,3)*Mat(3,i);
end


Mat = Mat1;
for i=1:1:6
	Mat1(1,i) = Mat(1,i)-Mat(1,2)/Mat(2,2)*Mat(2,i);
end


Mat = Mat1;
for i=1:1:6
    Mat1(3,i) = Mat(3,i)-Mat(3,2)/Mat(2,2)*Mat(2,i);
end



Mat = Mat1;
for i=1:1:6
    Mat1(2,i) = Mat(2,i)-Mat(2,1)/Mat(1,1)*Mat(1,i);
end

Mat = Mat1;
for i=1:1:6    
    Mat1(3,i) = Mat(3,i)-Mat(3,1)/Mat(1,1)*Mat(1,i);
end


Mat=Mat1;
for i=1:1:3
    Mat1(i,:) = Mat(i,:)/Mat(i,i);
end


MatInv = Mat1(1:3,4:6)
inv(Mat3)

fprintf('===================Method 1==============');
Mat1 = [Mat3 eye(3)]
MatS = Mat1(1:3,1:3);

for k = 3:-1:1
       
for j = 1:1:3
        
if (j ~= k)

Mat = Mat1;
for i=1:1:6               
	Mat1(j,i) = Mat(j,i)-Mat(j,k)/Mat(k,k)*Mat(k,i);
end

end
        
        
end
end
Mat=Mat1;
for i=1:1:3
    Mat1(i,:) = Mat(i,:)/Mat(i,i);
end

MatInv = Mat1(1:3,4:6)
inv(Mat3)

%%
fprintf('===================Method 3==============');

Mat1 =  [2 -1 0.0
 -1 2 -1
 0.0 -1 2]

for i=1:1:3
    for k=1:1:3

    Mat2(i*6+k-6) = Mat1(i*3+k-3);

    if (k==i)          
       Mat2(i*6+3+k-6) = 1.0;
    else
       Mat2(i*6+3+k-6) = 0.0;
    end
    end       
end

fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n', Mat2(1), Mat2(2), Mat2(3), Mat2(4), Mat2(5),Mat2(6));
fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n', Mat2(7), Mat2(8), Mat2(9),Mat2(10),Mat2(11),Mat2(12));
fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n',Mat2(13),Mat2(14),Mat2(15),Mat2(16),Mat2(17),Mat2(18));

for k = 3:-1:1       
for j = 1:1:3        
if (j ~= k)
Mat3 = Mat2;
for i=1:1:6               
	Mat2((j-1)*6+i) = Mat3((j-1)*6+i)-Mat3((j-1)*6+k)/Mat3((k-1)*6+k)*Mat3((k-1)*6+i);
end

end
        
        
end
end

Mat3 = Mat2;
fprintf('\n\n');
fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n', Mat2(1), Mat2(2), Mat2(3), Mat2(4), Mat2(5),Mat2(6));
fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n', Mat2(7), Mat2(8), Mat2(9),Mat2(10),Mat2(11),Mat2(12));
fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n',Mat2(13),Mat2(14),Mat2(15),Mat2(16),Mat2(17),Mat2(18));

for i=1:1:3
    for j=1:1:6
        Mat2((i-1)*6+j) = Mat3((i-1)*6+j)/Mat3((i-1)*6+i);
    end
end
fprintf('\n\n');

fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n', Mat2(1), Mat2(2), Mat2(3), Mat2(4), Mat2(5),Mat2(6));
fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n', Mat2(7), Mat2(8), Mat2(9),Mat2(10),Mat2(11),Mat2(12));
fprintf('%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f\n',Mat2(13),Mat2(14),Mat2(15),Mat2(16),Mat2(17),Mat2(18));


Mat3 = Mat2;
for i=1:1:3
    for j=1:1:3
        Mat5((i-1)*3+j) = Mat3((i-1)*6+3+j);
    end
end


fprintf('\n\n');

fprintf('%4.3f %4.3f %4.3f\n', Mat5(1), Mat5(2), Mat5(3));
fprintf('%4.3f %4.3f %4.3f\n', Mat5(4), Mat5(5), Mat5(6));
fprintf('%4.3f %4.3f %4.3f\n', Mat5(7), Mat5(8), Mat5(9));
