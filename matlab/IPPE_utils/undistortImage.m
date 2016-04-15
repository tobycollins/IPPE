function I2 = undistortImage(I,K,kc)
% function to undo lens distortion with perspective camera intrinsics K and kc
% This is the model from Oulu university and described clearly in Bouguet's Matlab camera calibration toolbox
% (http://www.vision.caltech.edu/bouguetj/calib_doc/). This is done using
% Bouguet's toolbox (rect). You need to download it.

if ~exist('rect.m','file')
    error('To undistort an image you need to download Bouguets Matlab camera calibration toolbox (http://www.vision.caltech.edu/bouguetj/calib_doc/) and add it to the path.')
end
if isa(I,'uint8')
   I = im2double(I); 
end
for i=1:size(I,3)   
    I2(:,:,i) =rect((I(:,:,i)),eye(3),[K(1,1),K(2,2)],[K(1,3),K(2,3)],kc,K(1,2)/K(1,1),K);   
end
I2 = uint8(I2*255);

