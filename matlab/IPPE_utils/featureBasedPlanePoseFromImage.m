function [IPPEPoses,refinedPoses,HHat,U,Q] = featureBasedPlanePoseFromImage(planarTemplate,inputImage,K,kc,featureOpts,ransacOpts,IPPEOpts,plottingOpts)
%featureBasedPlanePoseFromImage: An example of using IPPE to solve a plane's pose given a frontal 'template' image and a single input image. This follows the following pipeline:
%
%1. Detect features in the template and input image
%2. Match features based on their descriptor similarity
%3. Use the matched features to robustly estimate the homography between the template image and the input image.
%4. From the inlier matches, use IPPE to estimate the camera's pose.
%5. (Optionally) Refine the camera's pose with Levenberg–Marquardt.
%
% The camera should be intrinsically calibrated a priori. We use a perspective camera model with lens distortion, which
% is the standard model and used in OpenCV and Bouguet's Matlab camera calibration toolbox
%(http://www.vision.caltech.edu/bouguetj/calib_doc/). This is parameterised by a 3x3 calibration matrix K and a 5x1 distortion vector
%kc. You can determine K and kc by running Bouguet's calibration toolbox using a checkerboard calibration target.
%
%Inputs
%
%planarTemplate: a structure holding the planar object used to compute the
%camera's pose. See makePlanarTemplate.m for a description of its fields.
%
%inputImage: a 2D unit8 input image (grayscale or rgb). This is an image of the plane viewed
%from the camera. 
%
%K: The input image's 3x3 intrinsic matrix (5x1 double).
%
%kc: The input image's distortion parameters (5x1 double)
%
%Note that when using ASIFT, it is best to use an undistorted image (i.e.
%the effects of lens distortion are undone). This is because ASIFT does som
%match filtering based on epipolar geometry, and does not handle lens
%distortion. 
%
%featureOpts: Options for feature detection and matching. See
%detectAndMatchFeatures.m for details.
%
%ransacOpts: Options for ransac. See basicHomographyRansac.m for details.
%detectAndMatchFeatures.m for details.
%
%IPPEOpts: Options for IPPE pose estimation. See perspectiveIPPE.m for details.
%
%plottingOpts: Options for plotting the fitted homography. This has two
%fields:
%    plottingOpts.doPlot (true or false): true if we want to visualise
%    results, false otherwise
%    plottingOpts.figId (positive integer): the figure number for plotting the results. 
%
%Outputs:
%IPPEPoses: structure containing the two pose solutions from IPPE. See
%perspectiveIPPE.m for details.
%
%refinedPoses: structure containing the two refined pose solutions using
%Levenberg–Marquardt (same format as IPPEPoses). See
%perspectiveIPPE.m for details. 
%
%HHat is the homography matrix outputted from ransac (3x3 double). Note
%that this is the mapping from the template's image to the input image's
%normalised pixel coordinates. 
%
%poseResults.U is the set of inlier correspondences detected in the template image (in mm) (2xN double)
%
%poseResults.Q is the set of inlier correspondences detected in the input image (in normalised pixels) (2xN double)
%
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
%basic argument checking:
assert(isa(inputImage,'uint8'));
assert(size(K,1)==3);
assert(size(K,2)==3);
assert(size(kc,1) == 5);
assert(size(kc,2) == 1);

%first detect and match features:
[p,q] = detectAndMatchFeatures(planarTemplate.rectifiedImage_g,planarTemplate.roi,...
    inputImage,[],featureOpts);
qNormalised = normaliseImagePoints(q,K,kc);

%now estimate the homography with RANSAC:
[HHat,inliers] = basicHomographyRansac(p,qNormalised,ransacOpts);

%lets plot the results:
if plottingOpts.doPlot
    visualiseHomography(planarTemplate.rectifiedImage_g,planarTemplate.roi,inputImage,HHat,K,kc,p,q,inliers,plottingOpts.figId);   
end

%Now use IPPE to estimate the camera's pose.

%First get the points on the template image into mm, and reject outliers:
U = planarTemplate.templatImgPixelSize*p(:,inliers);
Q = qNormalised(:,inliers);

[IPPEPoses,refinedPoses] = perspectiveIPPE(U,Q,[],IPPEOpts);
