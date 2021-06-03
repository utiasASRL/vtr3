function desc = surf_descriptor( iimg, keypoint, Options )
% SURF_DESCRIPTOR Computes the SURF descriptor for the given keypoint. The
%                 lattice used to compute the descriptor is oriented such
%                 that the principal orientation points to the right side.
%
% desc = surf_descriptor( iimg, keypoint, Options )
%
% INPUTS:
% iimg     - the integral image
% keypoint - the interest point struct
% Options  - struct indicating configuration parameters
%
% OUTPUT:
% desc - the 64-element descriptor
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
DESC_LENGTH = 64;

%% Initialization
desc = zeros(DESC_LENGTH,1); % allocate memory for the descriptor

%% Precompute Haar Filters
haarX = [surf_iiAreaLookupsC(-keypoint.scale/2,                 0,   keypoint.scale, 2*keypoint.scale, -1, Options.integer_lookups); ...
         surf_iiAreaLookupsC( keypoint.scale/2,                 0,   keypoint.scale, 2*keypoint.scale,  1, Options.integer_lookups)];
haarY = [surf_iiAreaLookupsC(                0, -keypoint.scale/2, 2*keypoint.scale,   keypoint.scale, -1, Options.integer_lookups); ...
         surf_iiAreaLookupsC(                0,  keypoint.scale/2, 2*keypoint.scale,   keypoint.scale,  1, Options.integer_lookups)];

%% Precompute rotation values
cos_ori = cos(keypoint.orientation);
sin_ori = sin(keypoint.orientation);

%% Compute all of the filter responses
count = 0;
% Major blocks (4x4 - top-left corners)
for i = -10:5:5
    for j = -10:5:5
        % Reset sums
        dx = 0;
        dy = 0;
        abs_dx = 0;
        abs_dy = 0;
        
        % Sample points in each major block (5x5)
        for ii = i:(i+5-1)
            for jj = j:(j+5-1)
                % Add 0.5 to space out points properly
                % (otherwise, points are offset to the top and to the left)
                iii = ii + 0.5;
                jjj = jj + 0.5;
                
                % Compute rotated sample points
                % (clockwise rotation since we are rotating the lattice)
                sample_x = keypoint.x + ( jjj*keypoint.scale*cos_ori + iii*keypoint.scale*sin_ori);
                sample_y = keypoint.y + (-jjj*keypoint.scale*sin_ori + iii*keypoint.scale*cos_ori);
                
                % Compute Gaussian weighted x and y Haar responses
                g = gaussian2d(iii, jjj, 3.3);  % SURF appears to use a different value, but not sure what it is
                hx = g * surf_eval_mask(iimg, sample_x, sample_y, haarX);
                hy = g * surf_eval_mask(iimg, sample_x, sample_y, haarY);
                
                % Rotate the responses
                % (counterclockwise rotation as we're rotating back to zero orientation)
                % (approximately the same as rotating the image and computing the filters)
                rhx = hx*cos_ori - hy*sin_ori;
                rhy = hx*sin_ori + hy*cos_ori;
                
                % Add the responses at this sample point to those in the block
                dx = dx + rhx;
                dy = dy + rhy;
                abs_dx = abs_dx + abs(rhx);
                abs_dy = abs_dy + abs(rhy);
            end
        end
        
        % Store sums in descriptor (ordering matches SURF)
        desc(count + 1) = dx;
        desc(count + 2) = abs_dx;
        desc(count + 3) = dy;
        desc(count + 4) = abs_dy;
        count = count + 4;
    end
end

%% Convert to a unit vector
desc = desc / sqrt(sum(desc.^2));

end


%% 2D Gaussian Function
function g = gaussian2d(x, y, sig)
    g = 1/(2*pi*sig*sig) * exp( -(x*x+y*y)/(2*sig*sig));
end

