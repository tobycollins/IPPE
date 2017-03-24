function [IPPEPoses,refinedPoses] = perspectiveIPPE(U,Q,hEstMethod,opts)
%perspectiveIPPE: The solution to Perspective IPPE with point correspondences computed
%between points in world coordinates on the plane z=0, and normalised points in the
%camera's image.
%
%Inputs:
%
%U: 2xN or 3xN matrix holding the model points in world coordinates. If U
%is 2xN then the points are defined in world coordinates on the plane z=0
%
%Q: 2xN matrix holding the points in the image. These are in normalised
%pixel coordinates. That is, the effects of the camera's intrinsic matrix
%and lens distortion are corrected, so that the Q projects with a perfect
%pinhole model.
%
%hEstMethod: the homography estimation method, either 'Harker' (default) or 'DLT'. If
%'Harker' then the method of Harker and O'Leary is used, from the paper
%"Computation of Homographies" in the 2005 British Computer Vision
%Conference. Otherwise the Direct Linear Transform is used, from Peter Kovesi's implementation at http://www.csse.uwa.edu.au/~pk.
%If hEstMethod=[] then the default is used.
%
%opts is an optional datastructure holding the fields:
%setup IPPE options:
%opts.withPoseRefinement = true; %If this is set, we also refine the camera pose from IPPE by gradient-based optimisation (Levenberg–Marquardt). This is designed to give the statistically optimal pose assuming zero mean I.I.D nose for the feature's 2D positions, but requires initialisation (from IPPE). Note that it about two orders of magnitude slower than IPPE.
%opts.displayTiming = true; %set this to true if you want to see how long IPPE takes in Matlab.
%
%Outputs:
%IPPEPoses: structure containing the two pose solutions from IPPE.
%   IPPEPoses.R1 is the first pose's rotation
%   IPPEPoses.t1 is the first pose's translation

%   IPPEPoses.R2 is the second pose's rotation
%   IPPEPoses.t2 is the second pose's translation

%   IPPEPoses.reprojError1 is the first pose's reprojection error
%   IPPEPoses.reprojError2 is the second pose's reprojection error
%
%refinedPoses: structure containing the two refined pose solutions using
%Levenberg–Marquardt (same format as IPPEPoses). This is only outputted if opts.withPoseRefinement = true
%
%Note that the poses are transforms from world
%coordinates to camera coordinates. If you
%want to get the camera's pose in world coordinates you would get it with
%M1 = inv([[R1,t1];[0,0,0,1]]) and M2 = inv([[R2,t2];[0,0,0,1]])
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

opts = parseOpts(opts);

measureTiming = opts.measureTiming;
refineSolutions = opts.withPoseRefinement;


U = double(U);
Q = double(Q);

%set defaults for hEstMethod:
if isempty(hEstMethod)
    hEstMethod = 'Harker';
else
    if ~(strcmp(hEstMethod,'Harker')||strcmp(hEstMethod,'DLT'))
        error('You must specify a valid homography estimation method in perspectiveIPPE. The options are Harker or DLT');
    end
end

%argument sanity check:
assert(size(U,1)==2|size(U,1)==3);
assert(size(U,2)==size(Q,2));
assert(size(Q,1) == 2);

%n is the number of points:
n = size(U,2);
modelDims = size(U,1);


Uuncentred = U;



if modelDims==2
   %we zero-center the model points:
    Pbar = [mean(U,2);0];
    U(1,:) = U(1,:)-Pbar(1);
    U(2,:) = U(2,:)-Pbar(2);
else
    %we rotate the model points onto the plane z=0 and zero center them:
    Pbar = mean(U,2);   
    MCenter = eye(4);
    MCenter(1:3,end) = -Pbar;    
    U_ = MCenter(1:3,:)*[U;ones(1,size(U,2))];  
    %compute the rotation that rotates the model points onto the plane z=0: 
    [modelRotation,sigs,~] = svd(U_*U_'); modelRotation=modelRotation';
    if det(modelRotation)<0
        modelRotation(3,:)  = -modelRotation(3,:); %this ensures modelRotation is a member of SO3
    end
    if (sigs(3,3)/sigs(2,2)>1e-5)
        error('IPPE requires the model points to be planar!');
    end
    
    modelRotation(4,4) = 1;
    Mcorrective = modelRotation*MCenter;
    U = Mcorrective(1:2,:)*[U;ones(1,size(U,2))];  
    
end


if measureTiming
    tic;
end
%compute the model-to-image homography:
switch hEstMethod
    case 'DLT'
        H = homography2d([U;ones(1,n)],[Q;ones(1,n)]);
    case 'Harker'
        H = homographyHarker([U;ones(1,n)],[Q;ones(1,n)]);
end

%[ px, py, 1, 0, 0, 0, -px*qx, -py*qx]h = qx
%[ 0, 0, 0, px, py, 1, -px*qy, -py*qy]h = qy

%Compute the Jacobian J of the homography at (0,0):
H = H./H(3,3);
J(1,1) = H(1,1)-H(3,1)*H(1,3);
J(1,2) = H(1,2)-H(3,2)*H(1,3);
J(2,1) = H(2,1)-H(3,1)*H(2,3);
J(2,2) = H(2,2)-H(3,2)*H(2,3);

%Compute the two rotation solutions:
v = [H(1,3);H(2,3)];
[R1,R2] = IPPE_dec(v,J);

%Compute the two translation solutions:
t1_ = estT(R1,U,Q);
t2_ = estT(R2,U,Q);

if modelDims==2
    t1 = [R1,t1_]*[-Pbar;1];
    t2 = [R2,t2_]*[-Pbar;1];
else   
    M1 = [R1,t1_];
    M1(4,4) = 1;
    M2 = [R2,t2_];
    M2(4,4) = 1;
    M1 = M1*Mcorrective;
    M2 = M2*Mcorrective;
    R1 = M1(1:3,1:3);
    R2 = M2(1:3,1:3);
    t1 = M1(1:3,end);
    t2 = M2(1:3,end);   
end

[reprojErr1,reprojErr2] = computeReprojErrs(R1,R2,t1,t2,Uuncentred,Q);
%sort solutions:
if reprojErr1>reprojErr2
    [R1,R2,t1,t2,reprojErr1,reprojErr2] = swapSolutions(R1,R2,t1,t2,reprojErr1,reprojErr2);
end
IPPEPoses.R1 = R1;
IPPEPoses.t1 = t1;
IPPEPoses.R2 = R2;
IPPEPoses.t2 = t2;
IPPEPoses.reprojError1 = reprojErr1;
IPPEPoses.reprojError2 = reprojErr2;



%%with timing?
if measureTiming
    disp('Computing time to estimate homography for IPPE. Warning this slows things down, so turn it off when you are not benchmarking!');
    tic;
    %average timing over 100 runs:
    for i=1:100
        switch hEstMethod
            case 'DLT'
                H = homography2d([U;ones(1,n)],[Q;ones(1,n)]);
            case 'Harker'
                H = homographyHarker([U;ones(1,n)],[Q;ones(1,n)]);
        end
    end
    
    tm = toc;
    disp(['Time to estimate homography: ' num2str(1000*tm/100) 'ms']);
    
    disp('Computing time to solve IPPE given homography...');
    tic;
    %average timing over 100 runs:
    for i=1:100
        %Compute the Jacobian J of the homography at (0,0):
        H = H./H(3,3);
        J(1,1) = H(1,1)-H(3,1)*H(1,3);
        J(1,2) = H(1,2)-H(3,2)*H(1,3);
        J(2,1) = H(2,1)-H(3,1)*H(2,3);
        J(2,2) = H(2,2)-H(3,2)*H(2,3);
        %Compute the two rotation solutions:
        v = [H(1,3);H(2,3)];
        [R1,R2] = IPPE_dec(v,J);
        %Compute the two translation solutions:
        t1_ = estT(R1,U,Q);
        t2_ = estT(R2,U,Q);
        t1 = [R1,t1_]*[-Pbar;1];
        t2 = [R2,t2_]*[-Pbar;1];    
    end
    tm = toc;
    disp(['Time to solve IPPE given homography: ' num2str(1000*tm/100) 'ms']);
end


if modelDims==2
    Uuncentred(3,:) = 0;
end

if refineSolutions
    poseEst= pnpLMRefinement(Uuncentred,Q,R1,t1,'MLE');
    R1 = poseEst.R;
    t1 = poseEst.t;
    
    poseEst= pnpLMRefinement(Uuncentred,Q,R2,t2,'MLE');
    R2 = poseEst.R;
    t2 = poseEst.t;
    
    [reprojErr1,reprojErr2] = computeReprojErrs(R1,R2,t1,t2,Uuncentred,Q);
    %Sort the solutions:
    if reprojErr1>reprojErr2
        [R1,R2,t1,t2,reprojErr1,reprojErr2] = swapSolutions(R1,R2,t1,t2,reprojErr1,reprojErr2);
    end
    refinedPoses.R1 = R1;
    refinedPoses.t1 = t1;
    refinedPoses.R2 = R2;
    refinedPoses.t2 = t2;
    refinedPoses.reprojError1 = reprojErr1;
    refinedPoses.reprojError2 = reprojErr2;
    
    
    %%with timing?
    if measureTiming
        disp('Computing time to refine IPPE solutions with Levenberg Marquardt...');
        tic;
        %average timing over 100 runs:
        for i=1:100
            pnpLMRefinement(Uuncentred,Q,IPPEPoses.R1,IPPEPoses.t1,'MLE');
            pnpLMRefinement(Uuncentred,Q,IPPEPoses.R2,IPPEPoses.t2,'MLE');
        end
        tm = toc;
        disp(['Time to refine IPPE solutions with Levenberg Marquardt: ' num2str(1000*tm/100) 'ms']);
    end
    
    
end






function [reprojErr1,reprojErr2] = computeReprojErrs(R1,R2,t1,t2,U,Q)
%computeReprojErrs: Computes the reprojection errors for the two solutions
%generated by IPPE.

%transform model points to camera coordinates and project them onto the
%image:
if size(U,1)==2
    PCam1 = R1(:,1:2)*U;
    PCam2 = R2(:,1:2)*U(1:2,:);
else
    PCam1 = R1*U;
    PCam2 = R2*U;
end


PCam1(1,:) = PCam1(1,:) + t1(1);
PCam1(2,:) = PCam1(2,:) + t1(2);
PCam1(3,:) = PCam1(3,:) + t1(3);


PCam2(1,:) = PCam2(1,:) + t2(1);
PCam2(2,:) = PCam2(2,:) + t2(2);
PCam2(3,:) = PCam2(3,:) + t2(3);

Qest_1 = PCam1./[PCam1(3,:);PCam1(3,:);PCam1(3,:)];
Qest_2 = PCam2./[PCam2(3,:);PCam2(3,:);PCam2(3,:)];

%compute reprojection errors:
reprojErr1 = norm(Qest_1(1:2,:)-Q);
reprojErr2 = norm(Qest_2(1:2,:)-Q);


function t = estT(R,psPlane,Q)
%Computes the least squares estimate of translation given the rotation solution.
if size(psPlane,1)==2
    psPlane(3,:) =0;
end
%qq = homoMult(Kinv,pp')';
Ps = R*psPlane;

numPts = size(psPlane,2);
Ax = zeros(numPts,3);
bx = zeros(numPts,1);

Ay = zeros(numPts,3);
by = zeros(numPts,1);



Ax(:,1) = 1;
Ax(:,3) = -Q(1,:);
bx(:) = Q(1,:).*Ps(3,:) -  Ps(1,:);

Ay(:,2) = 1;
Ay(:,3) = -Q(2,:);
by(:) = Q(2,:).*Ps(3,:) -  Ps(2,:);

A = [Ax;Ay];
b = [bx;by];

AtA = A'*A;
Atb = A'*b;

Ainv = IPPE_inv33(AtA);
t = Ainv*Atb;

function [R1,R2,t1,t2,reprojErr1,reprojErr2] = swapSolutions(R1_,R2_,t1_,t2_,reprojErr1_,reprojErr2_)
%swaps the solutions:
R1 = R2_;
t1 = t2_;
reprojErr1 = reprojErr2_;

R2 = R1_;
t2 = t1_;
reprojErr2 = reprojErr1_;


function Ainv = IPPE_inv33(A)
%computes the inverse of a 3x3 matrix, assuming it is full-rank.
a11 = A(1,1);
a12 = A(1,2);
a13 = A(1,3);

a21 = A(2,1);
a22 = A(2,2);
a23 = A(2,3);

a31 = A(3,1);
a32 = A(3,2);
a33 = A(3,3);

Ainv = [[ a22*a33 - a23*a32, a13*a32 - a12*a33, a12*a23 - a13*a22];
    [ a23*a31 - a21*a33, a11*a33 - a13*a31, a13*a21 - a11*a23];
    [ a21*a32 - a22*a31, a12*a31 - a11*a32, a11*a22 - a12*a21]];
Ainv = Ainv./(a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31);





function opts = parseOpts(opts)
if ~isfield(opts,'measureTiming')
    opts.measureTiming = false;
end
if ~isfield(opts,'withPoseRefinement')
    opts.withPoseRefinement = true;
end
