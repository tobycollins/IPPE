function [ H, Lh ] = homographyHarker( DataA, DataB, LA, LB)
%
% Purpose : Computes the general projective transformation between two sets
% of 2D data using a linear algorithm (the homography).
%
% Uses (syntax) :
%   H = homography( DataA, DataB ) 
%
% Input Parameters :
%   DataA, DataB := 2D homogeneous data sets in matrix form (3xn)
%
% Return Parameters :
%   H := the 3x3 homography
%
% Description and algorithms:
%   The algorithm is based on the Direct Linear Transform (DLT) method
%   outlined in Hartley et al.  The method uses orthogonal projections of
%   matrices, such that the vanishing line is treated as the principal
%   component of the reduction.  In this manner, the statistical behaviour
%   of the errors in variables are treated uniformly, see Harker and
%   O'Leary 2005.
%
% References :
%   Harker, M., O'Leary, P., Computation of Homographies, to appear in
%   Proceedings of the British Machine Vision Conference 2005, Oxford,
%   England.
%   Hartley, R., Zisserman, A., Multiple View Geometry in Computer Vision,
%   Cambridge University Press, Cambridge, 2001
%
% Cite this as :
%
% Author : Matthew Harker
% Date : July 25, 2005
% Version : 1.0
%--------------------------------------------------------------------------
% (c) 2005, O'Leary, Harker, University of Leoben, Leoben, Austria
% email: automation@unileoben.ac.at, url: automation.unileoben.ac.at
%--------------------------------------------------------------------------
% History:
%   Date:           Comment:
%   July 25, 2005   Original Version 1.0
%--------------------------------------------------------------------------
%
% Check input parameters:
%
if nargin == 2
    %
    if ~is2DData( DataA ) | ~is2DData( DataB )
        %
        error('Input does not represent 2D data') ;
        %
    end
    %
    [mA,nA] = size( DataA ) ;
    [mB,nB] = size( DataB ) ;
    %
    if (nA ~= nB)
        %
        error('Data sets must be the same size') ;
        %
    end
    %
    %
elseif nargin == 4
    %
    if ~is2dcovdata( DataA, LA ) | ~is2dcovdata( DataB, LB )
        %
        error('Input data does not match') ;
        %
    end
    %
    [mA,nA] = size( DataA ) ;
    [mB,nB] = size( DataB ) ;
    %
    if (nA ~= nB)
        %
        error('Data sets must be the same size') ;
        %
    end
    %
    %
    error('Error propagation not implemented as of yet') ;
    %
else
    %
    error('Incorrect number of input parameters') ;
    %
end
%
% Normalize the input data:
%
if nargin == 2
    %
    [DataA,TA,TAi] = normalizeData( DataA ) ;
    [DataB,TB,TBi] = normalizeData( DataB ) ;
    %
elseif nargin == 4    
    %
    [DataA,TA,TAi,LA] = normalizeData( DataA, LA ) ;
    [DataB,TB,TBi,LB] = normalizeData( DataB, LB ) ;
    %
end
%
% Construct the orthogonalized design matrix :
%
C1 = -DataB(1,:) .* DataA(1,:) ;
C2 = -DataB(1,:) .* DataA(2,:) ;
%
C3 = -DataB(2,:) .* DataA(1,:) ;
C4 = -DataB(2,:) .* DataA(2,:) ;
%
mC1 = mean( C1 ) ;
mC2 = mean( C2 ) ;
mC3 = mean( C3 ) ;
mC4 = mean( C4 ) ;
%
Mx = [ C1' - mC1, C2' - mC2, -DataB(1,:)' ] ;
My = [ C3' - mC3, C4' - mC4, -DataB(2,:)' ] ;
%
Pp = pinv(DataA(1:2,:)') ;
%
Bx = Pp * Mx ;
By = Pp * My ;
%
D = [ Mx - DataA(1:2,:)'*Bx ;...
      My - DataA(1:2,:)'*By ] ;
%
% Find v_min and backsubstitute :
%
[U,S,V] = svd( D, 0 ) ;
%
h789 = V(:,end) ;
h12 = -Bx * h789 ;
h45 = -By * h789 ;
h3 = -[mC1, mC2] * h789(1:2) ;
h6 = -[mC3, mC4] * h789(1:2) ;
%
% Reshape vector h to matrix H, and transform :
%
H = reshape( [h12; h3; h45; h6; h789], 3, 3)' ;
%
H = TB * H * TAi ;
%
% Compute the covariance of the output :
%
%
%
% END
%

function [DataN, T, Ti, LN] = normalizeData( Data, L ) ;
%
% Purpose : Computes a set of data corresponding to the input data with its
% centroid subtracted, and scaled such that the root-mean-square distance
% to the origin is sqrt(2).  The transformation T, carries the scaled data
% back to its original form.  Optionally, the first order estimation of
% covariance matrices are computed.
%
% Uses (syntax) :
%   [DataN, T, LN] = normalizeData( Data, L )
%   [DataN, T] = normalizeData( Data )
%
% Input Parameters :
%   Data := a 3xn matrix of homogeneous points.
%   L    := is a 3x3 covariance matrix (all points have identical covariance), or
%           a 3x3xn array of n covariance matrices.
%
% Return Parameters :
%   DataN := mean-free data scaled s.t. d_RMS = sqrt(2)
%   T     := transformation to bring DataN to the Affine coordinates
%            corresponding to Data (NOTE: T*DataN is in affine coords).
%   LN    := the covariance of the scaled normalized data (size is
%            generally 2x2xn, due to the normalization)
%
% Description and algorithms:
%
% References :
%   Clarke, J.C., Modelling Uncertainty: A Primer, Dept. of Engineering
%   Science, Oxford University, Technical Report.
%
% Cite this as :
%
% Author : Matthew Harker
% Date : July 7, 2005
% Version :
%
% (c) 2005, Institute for Automation, University of Leoben, Leoben, Austria
% email: automation@unileoben.ac.at, url: automation.unileoben.ac.at
%
% History:
%   Date:           Comment:
%                   Original Version 1.0
%--------------------------------------------------------------------------
%
% Check input arguments :
%
if nargin == 1
    %
    if ~is2DData( Data )
        %
        error('Input does not represent 2D data') ;
        %
    end
    %
elseif nargin == 2
    %
    if ~is2dcovdata( Data, L )
        %
        error('Input data and covariances do not match') ;
        %
    end
    %
else
    %
    error('Not enough input arguments') ;
    %
end
%
s = Data(1,:) ;
t = Data(2,:) ;
u = Data(3,:) ;
%
x = s./u ;
y = t./u ;
%
xm = mean( x ) ;
ym = mean( y ) ;
%
xh = x - xm ;
yh = y - ym ;
%
n = length( xh ) ;
%
kappa = sum( xh.^2 + yh.^2 ) ;
%
beta = sqrt( 2*n / kappa ) ;
%
xn = beta * xh ;
yn = beta * yh ;
%
DataN = [ xn; yn; ones(size(xn)) ] ;
%
T = [ 1/beta,   0   , xm ;...
        0   , 1/beta, ym ;...
        0   ,   0   ,  1 ] ;
%
Ti = [ beta ,  0   , -beta * xm ; ...
        0   , beta , -beta * ym ; ...
        0   ,  0   ,       1    ] ;
%
%
%
if nargin == 1
    %
    % Just the data is needed.
    %
    if nargout == 4
        warning('No covariance matrix for the input data given.') ;
        LN = [] ;
    end
    %
    %
    %
elseif nargin == 2
    %
    % There is a 3x3 homogeneous covariance matrix given
    %
    if nargout == 4
        %
        % The 2x2 covariance matrix of the Euclidean output data is required
        %
        dim = length( size( L ) ) ;
        %
        % dim == 2 --> all covariance matrices are assumed the same
        % dim == 3 --> each point has a given covariance
        %
        % DEHOMOGENIZE
        %
        for k = 1:n
            %
            %
            % Jacobian for dehomogenizing:
            Jd = ( 1/u(k)^2 ) * [ u(k),  0  , -s(k) ;...
                                   0  , u(k), -t(k) ] ;
            %
            %
            if dim == 2
                LN(:,:,k) = Jd * L * Jd' ;
            else
                LN(:,:,k) = Jd * L(:,:,k) * Jd' ;
            end
            %
            % NOTE: If all data has the same covariance, after
            % dehomogenizing they will generally all have different
            % covariance matrices (as implemented here).  Only if the data
            % is already affine, and the covariance matrix is Euclidean
            % will the "normalized" points all have the same covariance.
            %
        end
        %
        % SUBTRACT MEAN, SCALE:
        % 
        % Jacobian for subtracting mean:
        Jm = [ (1-(1/n)),   0       , -1 ,  0 ;...
                  0     , (1-(1/n)) ,  0 , -1 ] ;
        %
        % Covariance of the mean value:
        Lm = (1/n^2) * [ sum(LN(1,1,:)), sum(LN(1,2,:)) ;...
                         sum(LN(2,1,:)), sum(LN(2,2,:)) ] ;
        %
        %
        Lkappa = 0 ;
        %
        for k = 1:n
            %
            % Subtract Mean Covariance
            %
            LN(:,:,k) = Jm * [ LN(:,:,k) , zeros(2,2) ; zeros(2,2) , Lm ] * Jm' ;
            %
            % Compute Covariance of 'kappa':
            %
            Jk = 2 * [ xh(k), yh(k) ] ;
            Lkappa = Lkappa + Jk*LN(:,:,k)*Jk' ;
            %
        end
        %
        %
        for k = 1:n
            %
            % Jacobian for scaling the data:
            %
            a = kappa^(-3/2) ;
            b = kappa^(-1/2) ;
            %
            Js = sqrt(2*n) * [ (b - a*xh(k)^2) , -a*xh(k)*yh(k)  , -(a*xh(k))/2 ;...
                                -a*xh(k)*yh(k) , (b - a*yh(k)^2) , -(a*yh(k))/2 ] ;
            %
            LN(:,:,k) = Js * [ LN(:,:,k), [0;0] ; [0,0] , Lkappa ] * Js' ; 
            %
            %
            %
        end
        %
        %
        %
    elseif nargout == 3
        %
        warning('No covariance matrix has been returned') ;
        %
    end
    %
    %
end
%
% END
%        
