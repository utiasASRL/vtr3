function [ maxima failed R] = surf_interpolate_maxima( fh, maximaPos )
% SURF_INTERPOLATE_MAXIMA Perform subpixel, subscale interpolation on a
%                         maxima point by a (first-order) approximation the
%                         local region (a parabola), and computing the 
%                         "true" maxima.
%
% [ maxima failed ] = surf_interpolate_maxima( fh, maximaPos )
%
% INPUTS:
% fh          - the 3D block of interest operator evaluations for this
%               octave
% maximaPos   - a keypoint structure denoting the 3D index of a possible
%               maxima in the octave stack
%
% OUTPUTS:
% maxima      - a keypoint structure with the updated (if it doesn't fail)
%               subpixel/subscale maxima point
% failed      - a flag specifying if the interpolation fails:
%               when the peak of the quadratic is far from the maxima
% R           - the covariance of the x/y location of this feature point.
%

%%
% Copyright (c) 2010, Paul Furgale and Chi Hay Tong
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are 
% met:
% 
% * Redistributions of source code must retain the above copyright notice, 
%   this list of conditions and the following disclaimer.
% * Redistributions in binary form must reproduce the above copyright 
%   notice, this list of conditions and the following disclaimer in the 
%   documentation and/or other materials provided with the distribution.
% * The names of its contributors may not be used to endorse or promote 
%   products derived from this software without specific prior written 
%   permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
% TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
% OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%

%% Maxima Coordinates
mx = maximaPos.x;         % x
my = maximaPos.y;         % y
ms = maximaPos.scale;     % scale

%% Jacobian
% First-order central differences
d = zeros(3,1);

d(1) = 0.5*(fh(my, mx + 1, ms) - fh(my, mx - 1, ms));   % dx
d(2) = 0.5*(fh(my + 1, mx, ms) - fh(my - 1, mx, ms));   % dy
d(3) = 0.5*(fh(my, mx, ms + 1) - fh(my, mx, ms - 1));   % ds

%% Hessian
% Using first-order approximation of the second derivatives
H = zeros(3,3);

% dxx
H(1,1) =    fh(my, mx + 1, ms)...
         -2*fh(my, mx    , ms)...
         +  fh(my, mx - 1, ms);

% dyy
H(2,2) =    fh(my + 1, mx, ms)...
         -2*fh(my    , mx, ms)...
         +  fh(my - 1, mx, ms);  

% dsskp 
H(3,3) =   fh(my, mx, ms + 1)...
        -2*fh(my, mx, ms    )...
        +  fh(my, mx, ms - 1);

% dxy
H(1,2) =  0.25 * (fh(my + 1, mx + 1, ms)...
         -        fh(my + 1, mx - 1, ms)...
         -        fh(my - 1, mx + 1, ms)...
         +        fh(my - 1, mx - 1, ms));
H(2,1) = H(1,2);    % dyx

% dxs
H(1,3) =  0.25 * (fh(my, mx + 1, ms + 1)...
         -        fh(my, mx - 1, ms + 1)...
         -        fh(my, mx + 1, ms - 1)...
         +        fh(my, mx - 1, ms - 1));
H(3,1) = H(1,3);    % dsx

% dys
H(2,3) =  0.25 * (fh(my + 1, mx, ms + 1)...
         -        fh(my - 1, mx, ms + 1)...
         -        fh(my + 1, mx, ms - 1)...
         +        fh(my - 1, mx, ms - 1)); 
H(3,2) = H(2,3);    % dsy

%% Solve for the "true" maxima
x = -H\d;

% Store the covariance of the feature point
Rp = inv(-H);
R = Rp(1:2,1:2);
% If the step makes sense (maxima is within the interpolation region), perform it
if ( abs(x(1)) < 1 && abs(x(2)) < 1 && abs(x(3)) < 1 )
    failed = false;
    maxima = maximaPos;
    maxima.x = mx + x(1);
    maxima.y = my + x(2);
    maxima.scale = ms + x(3);
else    % Otherwise, fail
    failed = true;
    maxima = maximaPos;
end

