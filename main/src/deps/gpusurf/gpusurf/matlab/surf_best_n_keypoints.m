function [ keypoints threshold ] = surf_best_n_keypoints( keypoints, N )
% SURF_BEST_N_KEYPOINTS Gets the best N keypoints from the keypoint array,
%                       sorted by strength.
%
% [ keypoints threshold ] = surf_best_n_keypoints( keypoints, N )
%
% INPUTS:
% keypoints - a struct array of keypoints
%             each keypoint has x, y, scale, strength, laplacian,
%             orientation, and descriptor
% N         - The target number of keypoints.
% 
% OUTPUTS:
% keypoints - The truncated array of keypoints sorted by strength
% threshold - The threshold required to get this number of keypoints
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

%% Sort the keypoints by strength
strengths = [keypoints.strength];
[S, I] = sort(strengths,2,'descend');
keypoints = keypoints(I);

%% Truncate the array of keypoints, and return
if (length(keypoints) > N)
    % Truncate the array of keypoints.
    keypoints = keypoints(1:N);
end

% Grab the threshold that would result in this number of keypoints.
threshold = keypoints(end).strength;

