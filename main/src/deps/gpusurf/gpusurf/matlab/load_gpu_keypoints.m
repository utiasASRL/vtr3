function [ keypoints gpudata ] = load_gpu_keypoints( keypoint_file )
% LOAD_GPU_KEYPOINTS Loads a GPU keypoint file, and converts it into a
%                    Matlab-compatible keypoint structure.
%
% [ keypoints gpudata ] = load_gpu_keypoints( keypoint_file )
%
% INPUT:
% keypoint_file - .key file output from the GPU
%
% OUTPUT:
% keypoints - a struct array of keypoints:
%             each keypoint has x, y, scale, strength, octave, laplacian,
%             orientation, octave, and descriptor
% gpudata - an array of the raw data from the .key file
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

%% Load the data and initialize
gpudata = importdata(keypoint_file);

K = size(gpudata,1);
kp = struct('x', -1, ...
            'y', -1, ...
            'scale', -1, ...
            'strength', -1, ...
            'octave', 0, ...
            'laplacian', -1, ...
            'orientation', 0, ...
            'descriptor', []);
        
keypoints = repmat(kp,K,1);

%% Convert into the Matlab datastructure
for i = 1:size(gpudata,1)
    keypoints(i).x = gpudata(i,1) + 1;
    keypoints(i).y = gpudata(i,2) + 1;
    keypoints(i).scale = gpudata(i,3);
    keypoints(i).strength = gpudata(i,4);
    keypoints(i).orientation = gpudata(i,5);
    keypoints(i).octave = gpudata(i,6) + 1;
    keypoints(i).laplacian = gpudata(i,7);
    keypoints(i).orientation_strength = gpudata(i,8);
    % The covariance of the feature point (x/y only)
    keypoints(i).R = eye(2)*(2^(keypoints(i).octave-1)); %[gpudata(i,9), gpudata(i,10); gpudata(i,10), gpudata(i,11)];
    keypoints(i).descriptor = gpudata(i,12:end)';
    if length(keypoints(i).descriptor) ~= 64 && length(keypoints(i).descriptor) ~= 0
        error('the keypoint descriptor was not length 64: length: %d',length(keypoints(i).descriptor));
    end
end

end

