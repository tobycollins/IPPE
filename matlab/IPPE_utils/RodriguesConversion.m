function R2 = RodriguesConversion(R1)
% Rodrigues - converts a Rodrigues rotation vector to rotation matrix or vice versa.
%
% Usage:
%           R2 = Rodrigues(R1)
%
% Input:
%           R1 : Rodrigues rotation vector (3x1 or 1x3) or rotation matrix (3x3).
%
% Output:
%           R2 : rotation matrix (3x3) or Rodrigues rotation vector (3x1 or 1x3).
%
% This code follows the algorithm given by
% [1] R. Hartley and A. Zisserman "Multiple View Geometry in Computer Vision,"
%     Cambridge, pp.583-585, 2003.
%
% Kim, Daesik
% Intelligent Systems Research Center
% Sungkyunkwan Univ. (SKKU), South Korea
% E-mail  : daesik80@skku.edu
% Homepage: http://www.daesik80.com
%
% July 2008  - Original version.


[r,c] = size(R1);

%% Rodrigues Rotation Vector to Rotation Matrix
if ((r == 3) && (c == 1)) || ((r == 1) && (c == 3))
    wx = [  0   -R1(3)  R1(2);
        R1(3)   0   -R1(1);
        -R1(2)  R1(1)   0   ];
    
    R1_norm = sqrt(R1(1)^2 + R1(2)^2 + R1(3)^2);
    
    if (R1_norm < eps)
        R2 = eye(3);
    else
        R2 = eye(3) + sin(R1_norm)/R1_norm*wx + (1-cos(R1_norm))/R1_norm^2*wx^2;
    end
    
    %% Rotation Matrix to Rodrigues Rotation Vector
elseif (r == 3) && (c == 3)
    w_norm = acos((trace(R1)-1)/2);
    if (w_norm < eps)
        R2 = [0 0 0]';
    else
        R2 = 1/(2*sin(w_norm))*[R1(3,2)-R1(2,3);R1(1,3)-R1(3,1);R1(2,1)-R1(1,2)]*w_norm;
    end
end


