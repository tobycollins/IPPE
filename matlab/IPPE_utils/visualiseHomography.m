function visualiseHomography(image1,image1ROI,image2,HHat,K,kc,p,q,inliers,figId)
%visualiseHomography: A simple function to visualise a fitted homogtaphy
%between two sets of points.
%
%Inputs
%
%image1: The first image (2D unit8, grayscale or rgb). 
%
%image1ROI: A region-of-interest specifying which region(s) in image1 we
%have detected features in. If this is empty then the whole image is
%assumed.
%
%image2: The second image (2D unit8, grayscale or rgb). 
%
%HHat: (3x3 matrix) mapping points in image1 to their positions in image2
%(in normalised pixel coordinates). 
%
%
%K: The input image's 3x3 intrinsic matrix (5x1 double).
%
%kc: The input image's distortion parameters (5x1 double)
%
%p : 2XN matrix holding the matched points in image1
%q : 2XN matrix holding the matched points in image2
%
%inliers: Nx1 binary vector specifying which points are classed as inliers
%and which ones are classed as outliers. If inliers(i)=true, then the ith
%point is classed an inlier, otherwise it is classed an outlier.
%
%figId: positive integer giving the figure number for showing the
%homography.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of the IPPE package for fast plane-based pose
% estimation from the paper "Infinitesimal Plane-based Pose Estimation" by Toby Collins and Adrien Bartoli,
% published in the International Journal of Computer Vision, September
% 2014. A copy of the author's pre-print version can be found here:
%
% http://isit.u-clermont1.fr/~ab/Publications/Collins_Bartoli_IJCV14.pdf
%
% This package is free and covered by the BSD licence without any warranty.
% (c) Toby Collins 2015
%
% We hope you find this code useful and please cite our paper in your work:
%
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

%basic argument checking:
if nargin~=10
    error('visualiseHomography has ten input arguments');
end
assert(size(p,1)==2);
assert(size(q,1)==2);
assert(size(p,2)==size(q,2));
assert(figId>0);
assert(isa(inliers,'logical'));
assert(length(inliers)==size(p,2));

if isempty(image1ROI)
    image1ROI = true(size(image1,1),size(image1,2));
end

figure(figId);
clf;
subplot(1,2,1);
imshow(image1);
hold on;
plot(p(1,inliers),p(2,inliers),'g.','markersize',30);
plot(p(1,~inliers),p(2,~inliers),'r.','markersize',30);

subplot(1,2,2);
imshow(image2);
hold on;
plot(q(1,inliers),q(2,inliers),'g.','markersize',30);
plot(q(1,~inliers),q(2,~inliers),'r.','markersize',30);
subplot(1,2,1);


%lets also plot the bounary of the template in the input image
%according to the estimate homography:
bb = bwperim(image1ROI);
[i1,i2] = find(bb);
boundaryPtsTemplate = [i2';i1'];
boundaryPtsTemplate(3,:) = 1;
qBoundaryNormalised = HHat*boundaryPtsTemplate;
qBoundaryNormalised(1,:) = qBoundaryNormalised(1,:)./qBoundaryNormalised(3,:);
qBoundaryNormalised(2,:) = qBoundaryNormalised(2,:)./qBoundaryNormalised(3,:);

qBoundaryUnNormalised = unnormaliseImagePoints(qBoundaryNormalised(1:2,:),K,kc);
subplot(1,2,1);
plot(boundaryPtsTemplate(1,:),boundaryPtsTemplate(2,:),'c.');
subplot(1,2,2);
plot(qBoundaryUnNormalised(1,:),qBoundaryUnNormalised(2,:),'c.');
title({['Matched features between the first image (left) and the second image (right)'],'Green points show inliner matches and red points show outliers.','Cyan points show the region-of-interest of the first image mapped to the input image by the best-fitting homography.'});
set(gcf,'units','normalized','outerposition',[0 0 1 1]);