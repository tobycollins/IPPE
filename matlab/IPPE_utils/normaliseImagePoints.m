function qNormalised = normaliseImagePoints(q,K,kc)
%function to normalise a set of image points according to the perspective
%intrinsic calibration matrix K and distortion parameters kc. This is the model
%from Oulu university and described clearly in Bouguet's Matlab camera calibration toolbox
% (http://www.vision.caltech.edu/bouguetj/calib_doc/). This is done using
% code from Bouguet's toolbox.
%
%inputs:
%q: a 2*N matrix of un-normalised points in an image.
%
%K: a 3x3 perspective intrinsic calibration matrix
%
%kc: a 5x1 vector of radial and tangential distortion coefficients.
%
%outputs:
%qNormalised: the normalised points
%

Kinv = inv(K);
q(3,:) = 1;
qNormalised = Kinv*q;
qNormalised(1,:) = qNormalised(1,:)./qNormalised(3,:);
qNormalised(2,:) = qNormalised(2,:)./qNormalised(3,:);
qNormalised = qNormalised(1:2,:);

%now compensate for radial and tangential distortion:

if norm(kc)>eps
    k1 = kc(1);
    k2 = kc(2);
    k3 = kc(5);
    p1 = kc(3);
    p2 = kc(4);
    
    xd = qNormalised;
    x = xd; 				% initial guess
    
    for kk=1:20,
        
        r_2 = sum(x.^2);
        k_radial =  1 + k1 * r_2 + k2 * r_2.^2 + k3 * r_2.^3;
        delta_x = [2*p1*x(1,:).*x(2,:) + p2*(r_2 + 2*x(1,:).^2);
            p1 * (r_2 + 2*x(2,:).^2)+2*p2*x(1,:).*x(2,:)];
        x = (xd - delta_x)./(ones(2,1)*k_radial);
        
    end;
    qNormalised = x;
end
