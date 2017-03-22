function [R1,R2,gamma] = IPPE_dec(v,J)
%IPPE_dec: Computes the two solutions to rotation from the J Jacobian of the
%model-to-plane homography H (IPPE Algorithm 1).
%
%
%Inputs v: 2x1 vector holding the point in normalised pixel coordinates
%which maps by H^-1 to the point (0,0,0) in world coordinates.
%
%J: 2x2 Jacobian matrix of H at (0,0).
%
%Outputs:
%R1: 3x3 Rotation matrix (first solution)
%R2: 3x3 Rotation matrix (second solution)
%gamma: The positive real-valued inverse-scale factor.
%
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of the IPPE package for very fast plane-based pose
% estimation from a single perspective image, from the paper "Infinitesimal Plane-based Pose Estimation"
% by Toby Collins and Adrien Bartoli, published in the International Journal of Computer Vision, September 2014.
% A copy of the author's pre-print version can be found here: http://isit.u-clermont1.fr/~ab/Publications/Collins_Bartoli_IJCV14.pdf
%
% Feel free to contact Toby (toby.collins@gmail.com) if you have any
% questions about the paper and IPPE.
%
% This package is free and covered by the BSD licence without any warranty. We hope you find this code useful and please cite our paper in your work:
% (c) Toby Collins 2015
%
%@article{
%year={2014},
%issn={0920-5691},
%journal={International Journal of Computer Vision},
%volume={109},
%number={3},
%doi={10.1007/s11263-014-0725-5},
%title={Infinitesimal Plane-Based Pose Estimation},
%url={http://dx.doi.org/10.1007/s11263-014-0725-5},
%publisher={Springer US},
%keywords={Plane; Pose; SfM; PnP; Homography},
%author={Collins, Toby and Bartoli, Adrien},
%pages={252-286},
%language={English}
%}

%compute the correction rotation Rv:
t = norm(v);
if t<=eps
    %the plane is fronto-parallel to the camera, so set the corrective rotation Rv to identity. There will be only one solution to pose.
    Rv = eye(3)
else
    %the plane is not fronto-parallel to the camera, so set the corrective rotation Rv
    s = norm([v;1]);
    costh = 1/s;
    sinth = sqrt(1-1/(s^2));
    Kcrs = 1/t*[zeros(2,2),v;-v',0];
    Rv = eye(3) + sinth*Kcrs + (1-costh)*Kcrs^2;
end

%setup the 2x2 SVD decomposition:
B = [eye(2),-v]*Rv(:,1:2);

dt = (B(1,1)*B(2,2) - B(1,2)*B(2,1));
Binv = [B(2,2)/dt, -B(1,2)/dt;-B(2,1)/dt, B(1,1)/dt];

A = Binv*J;

%compute the largest singular value of A:
AAT = A*transpose(A);
gamma = sqrt(1/2*(AAT(1,1) + AAT(2,2) + sqrt((AAT(1,1)-AAT(2,2))^2 + 4*AAT(1,2)^2)));

%reconstruct the full rotation matrices:
R22_tild = A/gamma;
h = eye(2)-R22_tild'*R22_tild;
b = [sqrt(h(1,1));sqrt(h(2,2))];
if h(1,2)<0
    b(2) = -b(2);
end
d = IPPE_crs([R22_tild(:,1);b(1)],[R22_tild(:,2);b(2)]);
c = d(1:2);
a = d(3);
R1 = Rv*[R22_tild,c;b',a];
R2 = Rv*[R22_tild,-c;-b',a];


function v3 = IPPE_crs(v1,v2)
%3D cross product between vectors v1 and v2
v3 = zeros(3,1);
v3(1) = v1(2)*v2(3)-v1(3)*v2(2);
v3(2) = v1(3)*v2(1)-v1(1)*v2(3);
v3(3) = v1(1)*v2(2)-v1(2)*v2(1);






