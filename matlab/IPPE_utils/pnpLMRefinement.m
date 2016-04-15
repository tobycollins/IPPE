function poseEst= pnpLMRefinement(U,Q,RInit,tInit,errorFun)
%An implementation of PnP by iterative nonlinear refinement with levenberg marquardt. 
%This requires an initial solution (RInit for rotation and tInit for translation).
%
% (c) Toby Collins 2015
%
%Inputs:
% U: a 3xN matrix of source point correspondences in world coordinates.
%
% Q: a 2xN matrix of normalised point correspondences in the image.
%
% RInit: initial solution for the camera's rotation (3x3 matrix)
%
% tInit: initial solution for the camera's translation (3x1 vector)
%
%Note that poses (R,t) are for the transform from world
%coordinates to camera coordinates. If you
%want to get the camera's pose in world coordinates you would get it with
%M = inv([[R,t];[0,0,0,1]])
%
%
% errorFun (optional): The error function used. This is either 'MLE' (default - the Maximum
% Likelihood error, or equivantly the reprojection error), or 'STE' (the
% Symmetric Transfer Error). One should usually use the STE when there is
% similar noise in both U and Q. For a more detailed discussion about the
% STE and the MLE, please refer to H&Z's Multiview Geometry. http://www.robots.ox.ac.uk/~vgg/hzbook/
%
%outputs:
%poseEst: this has two fields, R (rotation) and t (translation)

%basic argument checking
assert(size(U,1) ==3);
assert(size(Q,1) ==2);

assert(size(U,2) ==size(Q,2));

if nargin < 3
    errorFun = 'MLE';
end

assert(strcmp(errorFun,'MLE')|strcmp(errorFun,'STE'));

%use rodrigues' formula to convert R to a rotation vector:
rV = RodriguesConversion(RInit);

%stack unknowns:
x0 = [rV;tInit];
OP = optimset;
if strcmp(errorFun,'MLE')
    OP.Jacobian = 'on';
    %jacobians for STE are not currently implemented.
end
OP.Display = 'off';

%run levenberg marquardt:
x = lsqnonlin(@(x)refineErr(x,U,Q,errorFun),x0,[],[],OP);

%extract poses:
rV = x(1:3);
t = x(4:6);
R = RodriguesConversion(rV');
poseEst.R = R;
poseEst.t = t;


function [r,J] = refineErr(x,ps,psPix_,errorFun)
rV = x(1:3);
t = x(4:6);
R = RodriguesConversion(rV);

switch errorFun
    case 'MLE'
        ps3D = R*ps;
        ps3D(1,:) = ps3D(1,:)+t(1);
        ps3D(2,:) = ps3D(2,:)+t(2);
        ps3D(3,:) = ps3D(3,:)+t(3);
        
        px = ps3D(1,:)./ps3D(3,:);
        py = ps3D(2,:)./ps3D(3,:);
        r = [px-psPix_(1,:);py-psPix_(2,:)];
        r = r';
        r = r(:);
        
        J = computeJacobians(x,R,ps);
        
    case 'STE'
        H = [R(:,1:2),t];
        HInv = inv(H);
        
        ps_h = ps;
        ps_h(3,:) = 1;
        qq = H*ps_h;
        xforward = qq(1,:)./qq(3,:);
        yforward = qq(2,:)./qq(3,:);
        
        r1 = [xforward-psPix_(1,:);yforward-psPix_(2,:)];
        r1 = r1';
        r1 = r1(:);
        
        psPix_h = psPix_;
        psPix_h(3,:) = 1;
        qq = HInv*psPix_h;
        xrev = qq(1,:)./qq(3,:);
        yrev = qq(2,:)./qq(3,:);
        
        r2 = [xrev-ps(1,:);yrev-ps(2,:)];
        r2 = r2';
        r2 = r2(:);
        
        r = [r1;r2];
end


function J = computeJacobians(x,R,ps)
%computation of residual error jacobians with respect to x. This was done
%using the matlab symbolic toolbox.
%
%Inputs:
%x (6x1 vector) holding rotation (first three elements) and translation
%(last three elements)
% R rotation matrix corresponding to x(1:3)
%ps (2*N matrix) holding the transformed positions of each point in normalised pixel
%coordinates
%
%Output:
%J the 2N*6 Jacobian matrix
psx = ps(1,:)';
psy = ps(2,:)';
psz = ps(3,:)';

numPts = size(ps,2);

x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);
x5 = x(5);
x6 = x(6);
R11 = R(1,1);
R12 = R(1,2);
R13 = R(1,3);

R21 = R(2,1);
R22 = R(2,2);
R23 = R(2,3);

R31 = R(3,1);
R32 = R(3,2);
R33 = R(3,3);

zr = zeros(numPts,1);

jresidx_tvec = [ 1./(x6 + R31.*psx + R32.*psy + R33.*psz), zr, -(x4 + R11.*psx + R12.*psy + R13.*psz)./(x6 + R31.*psx + R32.*psy + R33.*psz).^2];

jresidy_tvec = [ zr, 1./(x6 + R31.*psx + R32.*psy + R33.*psz), -(x5 + R21.*psx + R22.*psy + R23.*psz)./(x6 + R31.*psx + R32.*psy + R33.*psz).^2];

rxR = [ psx./(x6 + R31.*psx + R32.*psy + R33.*psz), psy./(x6 + R31.*psx + R32.*psy + R33.*psz), psz./(x6 + R31.*psx + R32.*psy + R33.*psz), zr, zr, zr, -(psx.*(x4 + R11.*psx + R12.*psy + R13.*psz))./(x6 + R31.*psx + R32.*psy + R33.*psz).^2, -(psy.*(x4 + R11.*psx + R12.*psy + R13.*psz))./(x6 + R31.*psx + R32.*psy + R33.*psz).^2, -(psz.*(x4 + R11.*psx + R12.*psy + R13.*psz))./(x6 + R31.*psx + R32.*psy + R33.*psz).^2];

ryR = [ zr, zr, zr, psx./(x6 + R31.*psx + R32.*psy + R33.*psz), psy./(x6 + R31.*psx + R32.*psy + R33.*psz), psz./(x6 + R31.*psx + R32.*psy + R33.*psz), -(psx.*(x5 + R21.*psx + R22.*psy + R23.*psz))./(x6 + R31.*psx + R32.*psy + R33.*psz).^2, -(psy.*(x5 + R21.*psx + R22.*psy + R23.*psz))./(x6 + R31.*psx + R32.*psy + R33.*psz).^2, -(psz.*(x5 + R21.*psx + R22.*psy + R23.*psz))./(x6 + R31.*psx + R32.*psy + R33.*psz).^2];

jx1s = jx1(x1,x2,x3);
jx2s = jx2(x1,x2,x3);
jx3s = jx3(x1,x2,x3);

jresidx_rvec = rxR*[jx1s,jx2s,jx3s];
jresidy_rvec = ryR*[jx1s,jx2s,jx3s];

Jx = [jresidx_rvec,jresidx_tvec];
Jy = [jresidy_rvec,jresidy_tvec];
J = [Jx;Jy];


function jx1 = jx1(x1,x2,x3)
t2 = x2.^2;
t3 = x3.^2;
t4 = x1.^2;
t5 = t2+t3+t4;
t6 = sqrt(t5);
t7 = t2+t3;
t8 = cos(t6);
t9 = t8-1.0;
t10 = 1.0./t5;
t11 = sin(t6);
t12 = 1.0./t5.^(3.0./2.0);
t13 = 1.0./t5.^2;
t14 = t11.*t12.*x1.*x3;
t15 = t4.*t11.*t12.*x2;
t16 = t4.*t9.*t13.*x2.*2.0;
t17 = t3+t4;
t18 = t8.*t10.*x1.*x2;
t19 = t4.*t11.*t12.*x3;
t20 = t4.*t9.*t13.*x3.*2.0;
t21 = 1.0./sqrt(t5);
t22 = t4.*t11.*t12;
t23 = t9.*t13.*x1.*x2.*x3.*2.0;
t24 = t11.*t12.*x1.*x2.*x3;
t25 = t9.*t10.*x1.*2.0;
t26 = t2+t4;
jx1 = [t7.*t9.*t13.*x1.*-2.0-t7.*t11.*t12.*x1;t14+t15+t16-t9.*t10.*x2-t8.*t10.*x1.*x3;t18+t19+t20-t9.*t10.*x3-t11.*t12.*x1.*x2;-t14+t15+t16-t9.*t10.*x2+t8.*t10.*x1.*x3;t25-t9.*t13.*t17.*x1.*2.0-t11.*t12.*t17.*x1;t22+t23+t24-t11.*t21-t4.*t8.*t10;-t18+t19+t20-t9.*t10.*x3+t11.*t12.*x1.*x2;-t22+t23+t24+t11.*t21+t4.*t8.*t10;t25-t9.*t13.*t26.*x1.*2.0-t11.*t12.*t26.*x1];


function jx2 = jx2(x1,x2,x3)
t2 = x1.^2;
t3 = x2.^2;
t4 = x3.^2;
t5 = t2+t3+t4;
t6 = sqrt(t5);
t7 = cos(t6);
t8 = t7-1.0;
t9 = t3+t4;
t10 = 1.0./t5;
t11 = sin(t6);
t12 = 1.0./t5.^(3.0./2.0);
t13 = 1.0./t5.^2;
t14 = t11.*t12.*x2.*x3;
t15 = t3.*t11.*t12.*x1;
t16 = t3.*t8.*t13.*x1.*2.0;
t17 = t2+t4;
t18 = 1.0./sqrt(t5);
t19 = t11.*t18;
t20 = t3.*t7.*t10;
t21 = t8.*t13.*x1.*x2.*x3.*2.0;
t22 = t11.*t12.*x1.*x2.*x3;
t23 = t11.*t12.*x1.*x2;
t24 = t3.*t11.*t12.*x3;
t25 = t3.*t8.*t13.*x3.*2.0;
t26 = t8.*t10.*x2.*2.0;
t27 = t2+t3;
jx2 = [t26-t8.*t9.*t13.*x2.*2.0-t9.*t11.*t12.*x2;t14+t15+t16-t8.*t10.*x1-t7.*t10.*x2.*x3;t19+t20+t21+t22-t3.*t11.*t12;-t14+t15+t16-t8.*t10.*x1+t7.*t10.*x2.*x3;t8.*t13.*t17.*x2.*-2.0-t11.*t12.*t17.*x2;t23+t24+t25-t8.*t10.*x3-t7.*t10.*x1.*x2;-t19-t20+t21+t22+t3.*t11.*t12;-t23+t24+t25-t8.*t10.*x3+t7.*t10.*x1.*x2;t26-t8.*t13.*t27.*x2.*2.0-t11.*t12.*t27.*x2];

function jx3 = jx3(x1,x2,x3)
t2 = x1.^2;
t3 = x2.^2;
t4 = x3.^2;
t5 = t2+t3+t4;
t6 = sqrt(t5);
t7 = cos(t6);
t8 = t7-1.0;
t9 = t3+t4;
t10 = sin(t6);
t11 = 1.0./t5;
t12 = 1.0./t5.^(3.0./2.0);
t13 = 1.0./t5.^2;
t14 = 1.0./sqrt(t5);
t15 = t4.*t10.*t12;
t16 = t8.*t13.*x1.*x2.*x3.*2.0;
t17 = t10.*t12.*x1.*x2.*x3;
t18 = t8.*t11.*x3.*2.0;
t19 = t2+t4;
t20 = t7.*t11.*x2.*x3;
t21 = t4.*t10.*t12.*x1;
t22 = t4.*t8.*t13.*x1.*2.0;
t23 = t10.*t12.*x1.*x3;
t24 = t4.*t10.*t12.*x2;
t25 = t4.*t8.*t13.*x2.*2.0;
t26 = t2+t3;
jx3 = [t18-t8.*t9.*t13.*x3.*2.0-t9.*t10.*t12.*x3;t15+t16+t17-t10.*t14-t4.*t7.*t11;t20+t21+t22-t8.*t11.*x1-t10.*t12.*x2.*x3;-t15+t16+t17+t10.*t14+t4.*t7.*t11;t18-t8.*t13.*t19.*x3.*2.0-t10.*t12.*t19.*x3;t23+t24+t25-t8.*t11.*x2-t7.*t11.*x1.*x3;-t20+t21+t22-t8.*t11.*x1+t10.*t12.*x2.*x3;-t23+t24+t25-t8.*t11.*x2+t7.*t11.*x1.*x3;t8.*t13.*t26.*x3.*-2.0-t10.*t12.*t26.*x3];

