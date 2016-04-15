function [ptsImg1,ptsImg2] = asiftWrapper(asiftPath, img1,img2)
%asiftWrapper: A simple wrapper for calling ASIFT and reading the
%correspondences to matlab matrices. To use this you must install ASIFT and set its source path with the variable 
%asiftPath. Source is given at http://www.ipol.im/pub/art/2011/my-asift/,
%and you must compile it. This is very easy with cmake.
%
%
%Inputs asiftWrapper: The path to 
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

cc = cd;
cd(asiftPath);

imgOutVert = 'imgOutVert.png';
imgOutHori = 'imgOutHori.png';
matchings = 'matchings.txt';
keys1 = 'keys1.txt';
keys2 = 'keys2.txt';
flag_resize = 0;

imwrite(img1, './img1.png');
imwrite(img2, './img2.png');


demo_ASIFT('img1.png', 'img2.png', imgOutVert, imgOutHori, matchings, keys1, keys2, flag_resize);
[ptsImg1,ptsImg2] = readASIFTMatches('./matchings.txt');
% figure(1);
% clf;
% imshow(img1);
% hold on;
% plot(ptsImg1(1,:),ptsImg1(2,:),'r.');
% 
% figure(2);
% clf;
% imshow(img2);
% hold on;
% plot(ptsImg2(1,:),ptsImg2(2,:),'r.');

delete('./img1.png');
delete('./img2.png');

delete('./matchings.txt');
delete('./keys1.txt');
delete('./keys2.txt');
delete('./imgOutHori.png');
delete('./imgOutVert.png');

cd(cc);


function [ptsImg1,ptsImg2] = readASIFTMatches(fin)
fid=fopen(fin);
numMatches = str2double(fgetl(fid));
ptsImg1 = zeros(2,numMatches);
ptsImg2 = zeros(2,numMatches);

for i=1:numMatches
    tline = fgetl(fid);
    [p1xs,remain] = strtok(tline,' ');
    [p1ys,remain] = strtok(remain,' ');
    [p2xs,remain] = strtok(remain,' ');
    [p2ys,remain] = strtok(remain,' ');
    
    
    ptsImg1(1,i) = str2double(p1xs);
    ptsImg1(2,i) = str2double(p1ys);
    
    ptsImg2(1,i) = str2double(p2xs);
    ptsImg2(2,i) = str2double(p2ys);
    
end
fclose(fid);
