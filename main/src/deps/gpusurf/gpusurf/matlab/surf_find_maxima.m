function maxima = surf_find_maxima( hessian, laplacian, octave, Options )
% SURF_FIND_MAXIMA Finds the maxima within the hessian.
%
% maxima = surf_find_maxima( hessian, laplacian, octave, Options )
%
% INPUTS:
% hessian     - The interest operator (Hessian) evaluated at every position
%               and interval in the octave
% laplacian   - The Laplacian evaluated at every position and interval
% octave      - The internal parameters of this octave
% Options     - struct indicating configuration parameters
%
% OUTPUT: 
% maxima      - a struct array of keypoints:
%               each keypoint has x, y, scale, strength, octave, laplacian,
%               orientation, and descriptor
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
% Keypoint datastructures
kp = struct('x', -1, ...
            'y', -1, ...
            'scale', -1, ...
            'strength', -1, ...
            'octave', octave.number, ...
            'laplacian', -1, ...
            'orientation', 0, ...
            'descriptor', []);

% Preallocate memory for the maxima
maxima = repmat(kp,Options.max_feats_per_octave,1);
nMaxima = 0;

% Hessian dimensions
[SY SX SS] = size(hessian);

% Keypoint indexing
keypointOffset = 1;
if (Options.zero_indexed)
    keypointOffset = 0;
end

%% Seek maxima
% Iterate over the inner scales
for ss = 2:SS-1
    for sy = 2:SY-1
        for sx = 2:SX-1
            % Get the strength of the pixel
            vv = hessian(sy,sx,ss);
            
            if (abs(vv) > Options.threshold)    % If it is larger than the threshold
                % Check that this pixel strength is greater than all of its neighbours
                vvgtall = vv > hessian(sy-1:sy+1,sx-1:sx+1,ss-1:ss+1);
                
                if (sum(vvgtall(:)) >= 26)
                    failed = false;
                    nMaxima = nMaxima + 1;
                    lapl = laplacian(sy,sx,ss);
                    
                    % Store the keypoint parameters
                    maxima(nMaxima).x = sx;
                    maxima(nMaxima).y = sy;
                    maxima(nMaxima).scale = ss;
                    maxima(nMaxima).strength = vv;
                    maxima(nMaxima).laplacian = lapl;
                    
                    if (Options.do_subpixel_interpolation)     % Subpixel/subscale interpolation
                        [maxima(nMaxima) failed R] = surf_interpolate_maxima(hessian, maxima(nMaxima));
                        
                        if (failed)     % If the interpolation failed, drop the point
                            nMaxima = nMaxima - 1;
                        end
                        
                    end
                    
                    if (~failed)
                        % Transform the maxima into image space (from the octave/filter response stack)
                        maxima(nMaxima).x = ((maxima(nMaxima).x - 1) * octave.step) + octave.border + keypointOffset;
                        maxima(nMaxima).y = ((maxima(nMaxima).y - 1) * octave.step) + octave.border + keypointOffset;
                        maxima(nMaxima).scale = interp1(1:length(octave.scale), octave.scale, maxima(nMaxima).scale);
                        maxima(nMaxima).R = R * octave.step * octave.step;
                    end
                    
                end
                
            end
            
        end
    end
end

%% Return only the maxima found, or the first Options.max_feats_per_octave found
maxima = maxima(1:min(nMaxima,Options.max_feats_per_octave));
