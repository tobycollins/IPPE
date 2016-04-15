function qUnNormalised = unnormaliseImagePoints(qNormalised,K,kc)
%function to unnormalise a set of image points according to the perspective
%intrinsic calibration matrix K and distortion parameters kc. This is the model
%from Oulu university and described clearly in Bouguet's Matlab camera calibration toolbox
% (http://www.vision.caltech.edu/bouguetj/calib_doc/). This is done using
% code from Bouguet's toolbox.
%
%inputs:
%q: a 2*N matrix of normalised points in an image.
%
%K: a 3x3 perspective intrinsic calibration matrix
%
%kc: a 5x1 vector of radial and tangential distortion coefficients.
%
%outputs:
%qUnNormalised: the un-normalised points

%first compensate for radial and tangential distortion:
xn = qNormalised;
x = xn(1,:);
y = xn(2,:);

r = sqrt(xn(1,:).^2 + xn(2,:).^2);

dx(1,:) = 2*kc(3)*x.*y + kc(4)*(r.^2 + 2*x.^2);
dx(2,:) = kc(3)*(r.^2 + 2*y.^2) + 2*kc(4)*x.*y;


s = (1+kc(1)*r.^2 + kc(2)*r.^4 + kc(5)*r.^6);
xd = [s;s].*xn + dx;
xd(3,:) = 1;

%now apply the intrinsic matrix
qUnNormalised = K*xd;
qUnNormalised(1,:) = qUnNormalised(1,:)./qUnNormalised(3,:);
qUnNormalised(2,:) = qUnNormalised(2,:)./qUnNormalised(3,:);
qUnNormalised = qUnNormalised(1:2,:);
