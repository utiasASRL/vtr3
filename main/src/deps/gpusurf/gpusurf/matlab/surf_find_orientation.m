function ori = surf_find_orientation( iimg, keypoint, Options )
% SURF_FIND_ORIENTATION Determines the principal orientation of the
%                       interest point specified by (x,y,s).
%
% ori = surf_find_orientation( iimg, keypoint, Options )
%
% INPUTS:
% iimg     - the integral image
% keypoint - the interest point struct
% Options  - struct indicating configuration parameters
%
% OUTPUT:
% ori - the principal orientation, restricted to (-pi,pi)
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

%% Haar Filters
haarX = [surf_iiAreaLookupsC(-keypoint.scale,               0, 2*keypoint.scale, 4*keypoint.scale, -1, Options.integer_lookups); ...
         surf_iiAreaLookupsC( keypoint.scale,               0, 2*keypoint.scale, 4*keypoint.scale,  1, Options.integer_lookups)];
haarY = [surf_iiAreaLookupsC(              0, -keypoint.scale, 4*keypoint.scale, 2*keypoint.scale, -1, Options.integer_lookups); ...
         surf_iiAreaLookupsC(              0,  keypoint.scale, 4*keypoint.scale, 2*keypoint.scale,  1, Options.integer_lookups)];

%% Compute Haar Responses
if (Options.ori_restrict_to_circle)
    N = 113;	% 113 points fall inside a circle of radius 6s, step size s
else
    N = 169;
end

dx = zeros(N,1);
dy = zeros(N,1);
th = zeros(N,1);

idx = 0;
for i = -6:6
    for j = -6:6
        if ( (~Options.ori_restrict_to_circle) || ( (i*i + j*j) <= 36 ) )
            idx = idx + 1;
            
            % Compute filter center location
            sample_x = keypoint.x + i*keypoint.scale;
            sample_y = keypoint.y + j*keypoint.scale;
            
            % Compute weighted Haar responses
            g = gaussian2d(i,j,2);
            dx(idx) = g * surf_eval_mask(iimg, sample_x, sample_y, haarX);
            dy(idx) = g * surf_eval_mask(iimg, sample_x, sample_y, haarY);
            
            % Compute angle of vector formed
            th(idx) = atan2(dy(idx),dx(idx));
        end
    end
end

%% Compute dominant direction
if (Options.fast_orientation)
    %% Fast orientation computation
    ori = atan2(-sum(dy),sum(dx));
else
    %% Sliding window to find dominant direction
    % Seeking the window with the largest summed Haar wavelet response.
    r_max = 0;

    % Discretizing the search space by using each computed angle as one side of the window
    for i = 1:length(th)
        % Compute window boundaries by subtracting off the right-hand side of
        % the window, and re-adjusting so that the left-hand side doesn't wrap
        % around
        thm = anglemod(th - th(i));

        % Now all angles in this array with values greater than zero or
        % less than the window size are in bounds. We can then use this boolean
        % array to pick off the elements of th that are in the window
        inWindow = (thm >= 0) & (thm <= Options.ori_window_size);

        % Sum all of the dx and dy responses within the window
        dx_tot = sum( dx( inWindow ) );
        dy_tot = sum( dy( inWindow ) );

        % Compute the summed vector length
        r = sqrt(dx_tot*dx_tot + dy_tot*dy_tot);
        if (r > r_max)
            r_max = r;
            ori = atan2(-dy_tot,dx_tot);	% flip dy since positive y is down
        end
    end
end

end


%% 2D Gaussian Function
function g = gaussian2d(x, y, sig)
    g = 1/(2*pi*sig*sig) * exp( -(x*x+y*y)/(2*sig*sig));
end


%% Reduces an angle into the range -pi to pi
function a = anglemod(b)
    m2pi = 2*pi;
    a = b - (m2pi * round(b/m2pi));
end

