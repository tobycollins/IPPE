function planarTemplate = makePlanarTemplate(rectifiedImage,templatImgPixelSize,roi)
%This function builds a structure that holds all the data about the planar
%object (planarTemplate).
%
%(c) Toby Collins 2015
%
% Inputs:
%   rectifiedImage: a 2D uint8 template image (grayscale or rgb). This is an image of the plane viewed from directly above.
%   It should be an undistorted image with an aspect ratio of 1.
%
%   templatImgPixelSize: a scalar value which gives the size of the template image
%pixels in mm. That is, a distance of d pixels in the template image
%corresponds to a distance of d*templatImgPixelSize mm on the plane's
%surface.
%
%   (optional) templatImgROI: a 2D binary matrix of the same height and width as
%templatImg. This is used to specify the region in the template image that belong to the planar object.
%If templatImgROI(a,b) = true then pixel (a,b) belongs to the planar
%object, otherwise it does not.
%
%outputs
%planarTemplate: structure holding the planar template's data.
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

%basic argument checking:
assert(isa(rectifiedImage,'uint8'));
assert(size(rectifiedImage,3)==1|size(rectifiedImage,3)==3);
assert(templatImgPixelSize>0);

if nargin <3
   roi = true(size(rectifiedImage,1),size(rectifiedImage,2));
end
planarTemplate.rectifiedImage = rectifiedImage;
planarTemplate.templatImgPixelSize = templatImgPixelSize;
planarTemplate.roi = roi;
 

if size(rectifiedImage,3)==1
    planarTemplate.rectifiedImage_g = planarTemplate.rectifiedImage;
else
    planarTemplate.rectifiedImage_g = rgb2gray(planarTemplate.rectifiedImage);
end



