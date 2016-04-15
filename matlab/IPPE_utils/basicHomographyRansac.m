function [bestH,inliers] = basicHomographyRansac(p,q,ransacOpts)
%the basic RANSAC algorithm for estimating a homography from point
%correspondences with outliers.
%
%inputs:
%p a 2xN matrix of source point correspondences
%
%q a 2xN matrix of target point correspondences
%
%inlinerThresh: a scalar specifying the model's inlier threshold. This
%thredholds is used to class correspondences as inliers or outliers.
%Specifically, if we have computed the homography H and this transforms a
%source point p' to q', then it is considered an outlier if the distance between q' and the measured correspondence is
%greater than inlinerThresh.
%
%ransacOpts: options structure with the following fields:
%   maxSamples (a positive integer): This is the stopping criteria (a
%   reasonable value is 500, but this depends on the number of outliers).
%
%outputs:
%bestH: Best fitting homography that maps p to q.
%inliers: a binary Nx1 vector classifying each point as an inlier (true) or outlier (false).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%basic argument checking:
if nargin~=3
    error('basicHomographyRansac has three inputs');
end
if nargout>2
    error('basicHomographyRansac has two outputs');
end
if ~isfield(ransacOpts,'inlinerThresh')
    error('ransacOpts reqires the field: inlinerThresh');
end
if ~isfield(ransacOpts,'maxSamples')
    error('ransacOpts reqires the field: maxSamples');
end
assert(size(p,1)==2);
assert(size(q,1)==2);

assert(size(p,1)== size(q,1));

%make the points homogeneous
p(3,:) = 1;
q(3,:) = 1;

numPts = size(p,2);
bestNumInliers = 0;
bestH = [];
assert(numPts>=4);

%main ransac loop (nothing fancy here)
for i=1:ransacOpts.maxSamples
    %sample four points:
    s = randperm(numPts);
    s = s(1:4);
    p_ = p(:,s);
    q_ = q(:,s);
    
    %compute homography with the four points:
    try
        H = homographyHarker(p_,q_);
    catch
        %for badly configured points homographyHarker can fail, so catch
        %the error and continue.
        continue;
    end
    %evaulate the homography:
    pTransported = (H*p);
    pTransported(1,:) = pTransported(1,:)./pTransported(3,:);
    pTransported(2,:) = pTransported(2,:)./pTransported(3,:);
    resids = pTransported(1:2,:)-q(1:2,:);
    errs = sqrt(resids(1,:).^2 + resids(2,:).^2);
    numInliers = sum(errs<ransacOpts.inlinerThresh);
    %is it a better model?
    if numInliers>bestNumInliers
        bestNumInliers = numInliers;
        bestH = H;
        inliers = errs<ransacOpts.inlinerThresh;
    end
end
if isempty(bestH)
    error('RANSAC could not find a homography');
end
%now refine the homography with all inliers:
bestH = homographyHarker(p(:,inliers),q(:,inliers));
