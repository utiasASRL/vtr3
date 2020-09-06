function surf_plot_keypoints( image, keypoints, N )
% SURF_PLOT_KEYPOINTS Plots the detected keypoints on the specified image.
%
% surf_plot_keypoints( image, keypoints, N )
%
% INPUTS:
% image     - an MxN image matrix, or the file name of an image
% keypoints - the keypoints
% N         - (OPTIONAL) - plot the best N keypoints.
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
RADIUS_SCALE = 2.5;      % scaling factor applied to the keypoint scale to obtain the keypoint radius

%% Input check
if (nargin > 2)
    keypoints = surf_best_n_keypoints(keypoints,N);
end

%% Plot the points
% Load the image (if necessary)
hold off;
if (ischar(image))
    image = imread(image);
end

% Plot the points on the image
imagesc(image);
colormap gray;
hold on;
K = length(keypoints);

for k = 1:K
    % Colour based on Laplacian value
    if (keypoints(k).laplacian > 0)
        cc = 'r-';
    else
        cc = 'b-';
    end
    
    % Circles with arbitrarily scaled radius
    h = circle([keypoints(k).x,keypoints(k).y],keypoints(k).scale*RADIUS_SCALE,72,cc);
    set(h,'LineWidth',2);
    
    % Line indicating orientation (counterclockwise rotation)
    %line(keypoints(k).x + [0; keypoints(k).scale*RADIUS_SCALE*cos(keypoints(k).orientation)], ...
    %     keypoints(k).y + [0; -keypoints(k).scale*RADIUS_SCALE*sin(keypoints(k).orientation)], ...
    %     'Color',cc(1),'LineWidth',2);
end
axis image;

set(gca,'XTick',[]);
set(gca,'YTick',[]);
hold off;

