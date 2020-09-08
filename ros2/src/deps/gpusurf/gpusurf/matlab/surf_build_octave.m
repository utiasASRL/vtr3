function [ hessian laplacian ] = surf_build_octave( iimg, octave )
% SURF_BUILD_OCTAVE Builds the arrays of interest operator values evaluated
%                   over the image.
%
% [ hessian laplacian ] = surf_build_octave( iimg, octave )
%
% INPUTS:
% iimg   - the integral image
% octave - the struct containing this octave's mask specifications
%
% OUTPUTS:
% hessian     - a 3D array representing the Hessian evaluated
%               at every scale in the octave
%
% laplacian   - a 3D array representing the Laplacian evaluated at every
%               scale in the octave

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

%% Image and operation parameters
% Get the height and width of the original image
[H W] = size(iimg);

% Calculate the size of the stack based on the border and step size
xsize = floor((W - 2*octave.border)/octave.step);
ysize = floor((H - 2*octave.border)/octave.step);
ssize = length(octave.scale);

% Preallocate octave array
hessian = zeros(ysize,xsize,ssize);
laplacian = zeros(ysize,xsize,ssize);

%% Evaluate the interest operator at each scale in the octave
for ss = 1:ssize
    for sy = 1:ysize
        for sx = 1:xsize
            % Compute the lookup location
            % (sx + border is the location in the image, +1 for the Matlab-indexing)
            ix = (sx-1)*octave.step + octave.border + 1;
            iy = (sy-1)*octave.step + octave.border + 1;
            
            % Evaluate the box filter masks, and scale accordingly             
            Dxx = octave.DxxMult(ss) * surf_eval_mask(iimg,ix,iy,octave.dxx{ss});
            Dyy = octave.DyyMult(ss) * surf_eval_mask(iimg,ix,iy,octave.dyy{ss});
            Dxy = octave.DxyMult(ss) * surf_eval_mask(iimg,ix,iy,octave.dxy{ss});

            % Finally, calculate the interest operator at this location
            hessian(sy,sx,ss) = Dxx*Dyy - octave.edge_scale*(Dxy*Dxy);
            laplacian(sy,sx,ss) = (Dxx + Dyy) > 0;
        end
    end
end

