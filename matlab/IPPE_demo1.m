function IPPE_demo1
% IPPE_demo1: An example of using IPPE to solve a perspective camera's pose with a
% plane and a single image using ransac-based feature matching. 
%
% Two feature methods are supported (see detectAndMatchFeatures.m), which
% are affine-sift (ASIFT) and SURF. Other can be easily added. When the image of the plane is at a
% significant viewpoint angle (e.g. > 35 degrees), typically ASIFT does much better than standard
% methods such as SIFT and SURF, because they are not invariant to strong geometric image distortion.
%
% The images we demonstrate with are in
% ./examples/planarTemplate1/undistortedViews. These contain some very
% strong viewpoint angles, and SURF is not suitable for them.
%
% To solve this problem we require four things:
% a) A rectified 'template' image of the planar surface (for the demo example it is
% ./examples/planarTemplate1/templateImage.png). The template image can be made by taking a photo of the plane and then rectifying it manually with a homography.
%
% b) The scale of the template. This tells us how big the template's pixels
% are in mm, and is necessary for getting the camera's absolute translation (in mm).
%
% c) In input image. This is an image of the planar surface from an unknown
% viewpoint. The images for the demo are in ./examples/planarTemplate1/inputImages
%
% d) The camera calibration intrinsics. We use a perspective camera model with lens distortion, which
% is the standard model and used in OpenCV and Bouguet's Matlab camera calibration toolbox
% (http://www.vision.caltech.edu/bouguetj/calib_doc/). This is parameterised by a 3x3 calibration matrix K and a 5x1 distortion vector
% kc. You can determine K and kc by running Bouguet's calibration toolbox using a checkerboard calibration target.
% You can also compute it using Agisoft's Lens application, which is very
% nice and does not require any manual input. Make sure that the
% distortion coefficients (kc) are loaded in the correct order thought!!
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

% please modify the following to ASIFT's path:
asiftPath = '/home/toby/Downloads/demo_ASIFT_src';

disp('IPPE_demo1 is started!');
disp('Here we will use ransac-based feature matching to match a template image (./examples/planarTemplate1/templateImage.png) to five input images.');
disp('You must have installed ASIFT on your machine. Thuis is very easy - see http://www.ipol.im/pub/art/2011/my-asift/ for instructions.');
disp('For this demo the input images view the template from strong angles.');
disp('Generally ASIFT does a lot better for strong viewing angles than classical keypoint matching with e.g. SIFT or SURF.');
disp('Press any key to continue');

pause();

%setting the paths:
if exist(asiftPath,'dir')==0
   error('You must install ASIFT and set its source path with the variable asiftPath in IPPE_demo1.m See see http://www.ipol.im/pub/art/2011/my-asift/ for details');
end
addpath(genpath('./'));
addpath(genpath(asiftPath));

close all;

% setup the planar template:
templatImgPixelSize = 0.2;
templateROI = [];
planarTemplate = makePlanarTemplate(imread('./examples/planarTemplate1/templateImage.png'),templatImgPixelSize,templateROI);

% load a set of input images:
inputDir = './examples/planarTemplate1/inputImages/';
inputImages = dir([inputDir '*.JPG']);

K = load([inputDir '/K.mat']); K = K.K;
kc = load([inputDir '/kc.mat']); kc=kc.kc;

%setup feature matching options:
featureOpts.featureMethod = 'ASIFT';
featureOpts.asiftPath = asiftPath; %you must set the asift path here.

%featureOpts.featureMethod = 'SURF'; %careful, this only works well when the plane's viewpoint is not too tilted!!
%featureOpts.loweRatioThreshold = 1.2; %if using SURF features, you need to
%set a confidence ratio (see Lowe's SIFT paper for the explanation of
%this). Basically, a high value means only using feature matches that are
%likely to be correct (but at the cost of fewer feature matches). A default
%of 1.2 is usually fine.

%setup RANSAC options (see basicHomographyRansac.m):
ransacOpts.inlinerThresh = 0.0075; %if a match is predicted to with inlinerThresh pixels, then it is considered an inlier. This threshold is in normalised pixel coordinates, and corresponds to about 10 pixels for a 1080p image.
ransacOpts.maxSamples = 1000; %ransac terminates after maxSamples iterations.

%setup IPPE options:
IPPEOpts.withPoseRefinement = true; %If this is set, we also refine the camera pose from IPPE by gradient-based optimisation (Levenberg–Marquardt). This is designed to give the statistically optimal pose assuming zero mean I.I.D nose for the feature's 2D positions, but requires initialisation (from IPPE). Note that it about two orders of magnitude slower than IPPE.
IPPEOpts.measureTiming = true; %set this to true if you want to see how long IPPE takes in Matlab.

disp([featureOpts.featureMethod ' are used']);

%run IPPE with ransac-based feature selection:
for i=1:length(inputImages)
    disp(['Processing input image ' num2str(i) ' of ' num2str(length(inputImages)) ' using ' featureOpts.featureMethod ' features...']);
    
    %get the next image:
    inputImage = imread([inputDir inputImages(i).name]);
    
    plottingOpts.doPlot = true; %indicate we want to visualise the results:
    plottingOpts.figId = i; %have a figure for each image.
    
    %run IPPE:
    [IPPEPoses,refinedPoses] = featureBasedPlanePoseFromImage(planarTemplate,inputImage,K,kc,featureOpts,ransacOpts,IPPEOpts,plottingOpts);

    %lets get the IPPE pose with the lowest reprojection error and visualise it. By default this is the
    %first pose from IPPE (it always returns two poses). The second pose is
    %needed if the problem is ambiguous. This typically occurs if the plane
    %is small in the image and/or viewed from a large distance, which makes
    %the projection be quasi-affine. In these cases there is a flip
    %ambiguity. For more details about these ambiguities, please refer to
    %the IPPE paper.
    
    assert(IPPEPoses.reprojError1<=IPPEPoses.reprojError2);    
    M = [IPPEPoses.R1,IPPEPoses.t1];
    M(4,4) = 1;
        
    %Lets plot the plane's coordinates axes in the input image using M, to visulise the pose of the camera:
    if norm(kc)>eps
        inputImageUndistorted = undistortImage(inputImage,K,kc);
    else
        inputImageUndistorted = inputImage;
    end
    h = figure(i+length(inputImages));
    clf;
    subplot(1,2,1);
    plotPlaneAxes(M,inputImageUndistorted,K,100,h);
    title({'Visualisation of IPPE camera pose'});
    
    %lets get the refined pose (from Levenberg–Marquardt) with the lowes reprojection error and also
    %visualise it:
    assert(refinedPoses.reprojError1<=refinedPoses.reprojError2);    
    Mref = [refinedPoses.R1,refinedPoses.t1];
    Mref(4,4) = 1;
    
    h = figure(i+length(inputImages));
    subplot(1,2,2);
    plotPlaneAxes(Mref,inputImageUndistorted,K,100,h);
    title({'Visualisation of refined IPPE camera pose with Levenberg Marquardt'});
    set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    
    %lets compute the difference between the pose from IPPE and the one
    %after Levenberg–Marquardt refinement:
    Tdiff = (M(1:3,end)-Mref(1:3,end));
    TdiffRel = 100*norm(Tdiff)/norm(Mref(1:3,end));
    
    Rdiff = M(1:3,1:3)'*Mref(1:3,1:3);
    rvec = RodriguesConversion(Rdiff);
    ang = 360*norm(rvec)/(2*pi);
    
    subplot(1,2,1);
    xlabel(['The difference in estimated translation between IPPE and the refined translation is ' num2str(TdiffRel) '%.']);    
    subplot(1,2,2);
    xlabel(['The difference in estimated rotation between IPPE and the refined translation is ' num2str(ang) ' degrees.']);

    pause(0.1);
end

disp('IPPE_demo2 has finished!');
