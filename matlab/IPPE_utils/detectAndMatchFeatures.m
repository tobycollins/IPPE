function [p,q] = detectAndMatchFeatures(image1,image1ROI,image2,image2ROI,featureOpts)
%detectAndMatchFeatures: A wrapper for detecting and matching features
%between two images using various feature detection methods.

%Inputs
%
%image1: The first image (2D unit8, grayscale or rgb). 
%
%image1ROI: A region-of-interest specifying which region(s) in image1 we
%should detect features in. If image1ROI=[] then the whole image is used.
%
%image2: The second image (2D unit8, grayscale or rgb). 
%
%image2ROI: A region-of-interest specifying which region(s) in image2 we
%should detect features in. If image2ROI=[] then the whole image is used.
%
%featureOpts: Options field specifying the feature detection method.
%Currently only SURF (matlab's built-in) and ASIFT
%(http://www.ipol.im/pub/art/2011/my-asift/) are supported, but it is very
%easy to introduce others. 
%for ASIFT, featureOpts should have the structures:
%
%   featureOpts.featureMethod = 'ASIFT';
%   featureOpts.asiftPath (the path to the asift code & compiled executable).
%
%for SURF, featureOpts should have the structures:
%
%   featureOpts.featureMethod = 'SURF'; %careful, unlike ASIFT this only works well 
%when the plane's viewpoint is not too tilted!!
%
%   featureOpts.loweRatioThreshold = 1.2 (default) You need to
%set a confidence ratio (see Lowe's SIFT paper for the explanation of
%this). Basically, a high value means only using feature matches that are
%likely to be correct (but at the cost of fewer feature matches). A default
%of 1.2 is usually fine.
%
% outputs:
%p : 2XN matrix holding the matched points in image1
%q : 2XN matrix holding the matched points in image2


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
%
% This is free software covered by the FreeBSD License (see IPPE_license.txt) with Copyright (c) 2015 Toby Collins


%basic argument checking
if nargin~=5
   error('detectAndMatchFeatures has 5 input arguments.');
end
if nargout>2
   error('detectAndMatchFeatures has 2 output arguments.');
end

assert(nargin ==5);
assert(size(image1,3)==1|size(image1,3)==3);
assert(size(image2,3)==1|size(image2,3)==3);
assert(isa(image1,'uint8'));
assert(isa(image2,'uint8'));

%check whether regions-of-interest are used:
if isempty(image1ROI)
    image1ROI = true(size(image1,1),size(image1,2));
end
if isempty(image2ROI)
    image2ROI = true(size(image2,1),size(image2,2));
end

assert(isa(image1ROI,'logical'));
assert(isa(image1ROI,'logical'));


%convert image to grayscale:
if size(image1,3)==3
    image1 = rgb2gray(image1);
end

if size(image2,3)==3
    image2 = rgb2gray(image2);
end

%perform detection and matching:
switch featureOpts.featureMethod
    case 'SURF'
        %detection:
        pointsTemplate = detectSURFFeatures(image1,'MetricThreshold',200);
        featuresTemplate = extractFeatures(image1,pointsTemplate);
        p = pointsTemplate.Location';
        
        pointsInput = detectSURFFeatures(image2,'MetricThreshold',200);
        featuresInput = extractFeatures(image2,pointsInput);
        
        %matching using Lowe's ratio test for rejecting bad matches (see his SIFT paper for details)
        [IDX, D]= knnsearch(featuresInput,featuresTemplate,'K',2);
        vlds = D(:,2)./D(:,1)> featureOpts.loweRatioThreshold;
        q = pointsInput.Location(IDX(:,1),:)';
        q = q(:,vlds);
        p = p(:,vlds);
    case 'ASIFT'
        [p,q] = asiftWrapper(featureOpts.asiftPath,image1,image2);
    otherwise
        error('unknown feature detection method is specified');
end

%keep only features located in the rois:
vlds = interp2(double(image1ROI),p(1,:),p(2,:))==1;
vlds = vlds & interp2(double(image2ROI),q(1,:),q(2,:))==1;
p = p(:,vlds);
q = q(:,vlds);