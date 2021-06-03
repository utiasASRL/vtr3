function [ keypoints, iimg, img ] = surf_find_keypoints( img, Options )
% FIND_KEYPOINTS Finds SURF keypoints in an image.
%
% [ keypoints, iimg, img ] = surf_find_keypoints( img, Options )
%
% INPUTS:
% img     - an MxN image matrix, or the file name of an image
% Options - (optional) struct indicating configuration parameters
% 
%
% OUTPUTS:
% keypoints   - a struct array of keypoints:
%               each keypoint has x, y, scale, strength, octave, laplacian,
%               orientation, and descriptor
% iimg        - the MxN integral image used to compute the keypoints (does
%               not have zero-padding, just like the GPU implementation)
% img         - the original image
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

%% Configuration parameters
if (nargin < 2)
    Options = surf_init_options();          % initialize will all default parameters
else
    Options = surf_init_options(Options);	% fill in any missing fields
end

%% Find keypoints
[iimg img] = surf_build_iimg(img);          % Compute integral image
octaves = surf_init_fast_hessian(Options);	% Initialize fast hessian parameters

% Compute octaves and find maxima (keypoints)
keypoints = [];
for i = 1:length(octaves)
    [hessian, laplacian] = surf_build_octave(iimg, octaves(i));
    maxima = surf_find_maxima(hessian, laplacian, octaves(i), Options);
    keypoints = [keypoints; maxima];
end

% Truncate keypoints
keypoints = surf_best_n_keypoints(keypoints, Options.num_features);

% Describe keypoints
keypoints = surf_describe_keypoints(iimg, keypoints, Options);

