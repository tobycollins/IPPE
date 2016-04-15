function IPPE_demo2()
% IPPE_demo2: An example of using IPPE to solve a camera's pose with a plane and 500 simulated point correspondences.
% If all is working correctly with your installation then the pose from IPPE should have zero reprojection error (up to precision).
% We also compute the time taken by Matlab to solve IPPE with 500 correspondences.
%
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
%

addpath(genpath('./'));
disp('This is a test of IPPE using 500 simulated correspondences. If all is working correctly with your installation then the pose from IPPE should have zero reprojection error (up to precision).');
disp('We will also compute the time taken by Matlab to solve IPPE with 500 correspondences.');
disp('Started...');


%%generate a camera intrinsic matrix:
K = eye(3);
K(1:2,end) = [640/2;480/2];
K(1,1) = 640;
K(2,2) = 640;

%%generate some model points in world coordinates:
n = 1000;
U = rand(2,n);
U3d = U;
U3d(3,:) = 0;
[RModel,~,~] = svd(rand(3,3));
Uworld= RModel*U3d;



%%generate a rigid transform (R,t) from world coordinates to camera
%%coordinates, and transform model points to camera coordinates:
[R,S,V] = svd(rand(3,3));
PCam = R*Uworld;
c = mean(PCam,2);
t = [0.1;-0.1;2.5]-c;

PCam(1,:) = PCam(1,:) + t(1);
PCam(2,:) = PCam(2,:) + t(2);
PCam(3,:) = PCam(3,:) + t(3);

%%generate correspondences by projecting the points onto the image
Q = K*PCam;
Q(1,:) = Q(1,:)./Q(3,:);
Q(2,:) = Q(2,:)./Q(3,:);
Q = Q(1:2,:);

%perform IPPE:
homogMethod = 'Harker';  %Harker DLT 
Q_n = inv(K)*[Q;ones(1,size(Q,2))];
Q_n = Q_n(1:2,:);

opts.measureTiming = true;
opts.withPoseRefinement = true; 
[IPPEPoses,refinedPoses] = perspectiveIPPE(Uworld,Q_n,homogMethod,opts);


tol = 1e-6;
if (IPPEPoses.reprojError1<tol)
   disp('Test passed!'); 
    
else
   disp('Test failed!'); 
end
    
disp('Demo ended...');






