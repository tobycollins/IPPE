function plotPlaneAxes(M,undistortedImage,K,axesLength,h)
%A simple function to visualise the plane's coordinate axes in an input image.
%
%Inputs
%
%M: 4x4 world-to-camera rigid transform (the plane's template is defined in
%world coordinates on the plane z=0)
%
%undistortedImage: the undistorted image
%
%K: 3x3 camera intrinsic calibration matrix
%
%h: figure handle
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of the IPPE package for fast plane-based pose
% estimation from the paper "Infinitesimal Plane-based Pose Estimation" by Toby Collins and Adrien Bartoli,
% published in the International Journal of Computer Vision, September
% 2014. A copy of the author's pre-print version can be found here:
%
% http://isit.u-clermont1.fr/~ab/Publications/Collins_Bartoli_IJCV14.pdf
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

V0 = [0;0;0];
V1 = [axesLength;0;0];
V2 = [0;axesLength;0];
V3 = [0;0;-axesLength];
Vs = [V0,V1,V2,V3];title({'The coordinate axes of the plane are plotted in the input image to visualise the 3D pose of the camera'});

Vs(4,:) = 1;
VsCameraCoords = M*Vs;
VsImg = K*VsCameraCoords(1:3,:);
VsImg(1,:) = VsImg(1,:)./VsImg(3,:);
VsImg(2,:) = VsImg(2,:)./VsImg(3,:);
VsImg(3,:) = 1;
figure(h);
%clf;
imshow(undistortedImage);
hold on;
l1 = VsImg(1:2,1:2)';
l2 = VsImg(1:2,[1,3])';
l3 = VsImg(1:2,[1,4])';
plot(l1(:,1),l1(:,2),'r-','linewidth',4);
plot(l2(:,1),l2(:,2),'g-','linewidth',4);
plot(l3(:,1),l3(:,2),'b-','linewidth',4);