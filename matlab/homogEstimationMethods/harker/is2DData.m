function trueFalse = is2DData( Data )
%
% Purpose : Tests if the input argument represents valid 2D homogeneous
% coordinates.
%
% Uses (syntax) :
%   is2DData( Data )
%
% Input Parameters :
%   Data := the variable to be tested (should be 3xn, n greater than 0)
%
% Return Parameters :
%   trueFalse := 0 or 1 (false or true)
%
% Description and algorithms:
%   Tests the size of the input argument
%
% References :
%
% Cite this as :
%
% Author : Matthew Harker
% Date : July 13, 2005
% Version : 1.0
%--------------------------------------------------------------------------
% (c) 2005, O'Leary, Harker, University of Leoben, Leoben, Austria
% email: automation@unileoben.ac.at, url: automation.unileoben.ac.at
%--------------------------------------------------------------------------
% History:
%   Date:           Comment:
%   July 13, 2005   Original Version 1.0
%--------------------------------------------------------------------------
%
[m,n] = size( Data ) ;
%
if (m == 3) & (n > 0)
    %
    trueFalse = 1 ;
    %
else
    %
    trueFalse = 0 ;
    %
end

 
