function keypoints = surf_describe_keypoints( iimg, keypoints, Options )
% SURF_DESCRIBE_KEYPOINTS Computes SURF-64 descriptors for the keypoints.
%
% keypoints = surf_describe_keypoints( iimg, keypoints, Options )
%
% INPUTS:
% iimg      - the integral image
% keypoints - a struct array of keypoints:
%             each keypoint has x, y, scale, strength, octave, laplacian,
%             orientation, and descriptor
% Options   - struct indicating configuration parameters
%
%
% OUTPUT:
% keypoints - the augmented interest points with orientation and descriptor
%             struct entries filled in
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

%% Initialization
K = length(keypoints);	% number of keypoints

%% Keypoint Orientation
if (~Options.upright_descriptor)    % Otherwise, keep it at zero (upright)
    for k = 1:K
        keypoints(k).orientation = surf_find_orientation(iimg, keypoints(k), Options);
    end
end

%% Descriptor
for k = 1:K
    keypoints(k).descriptor = surf_descriptor(iimg, keypoints(k), Options);
end

