  function octaves = surf_init_fast_hessian( Options )
% SURF_INIT_FAST_HESSIAN Initializes the fast Hessian parameters.
%
% octaves = surf_init_fast_hessian( Options )
%
% INPUT:
% Options - struct indicating configuration parameters
%
% OUTPUT:
% octaves - a struct array containing the parameters for the fast Hessian
%            algorithm. The struct contains the following fields:
%            border  - the border size for this octave (in pixels)
%            step    - the step size to take in the image
%            lobe    - the filter lobe size
%            dxx     - an array of lookups for each Dxx mask
%            dyy     - an array of lookups for each Dyy mask
%            dxy     - an array of lookups for each Dxy mask
%            DxxMult - the values to multiply the Dxx filter responses by
%            DyyMult - the values to multiply the Dyy filter responses by
%            DxyMult - the values to multiply the Dxy filter responses by
%            scale   - the equivalent scale value that corresponds with the
%                      filter size
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

%% Filter parameters (for code clarity)
s = Options.s;
l1 = Options.l1;
l2 = Options.l2;
l3 = Options.l3;
l4 = Options.l4;
edge_scale = Options.edge_scale;

%% Initialize datastructures
proto =          struct('number', -1, ...
                        'border', -1, ...
                        'step', -1, ...
                        'lobe', zeros(Options.num_intervals,1), ...
                        'dxx', [], ...
                        'dyy', [], ...
                        'dxy', [], ...
                        'DxxMult', zeros(Options.num_intervals,1), ...
                        'DyyMult', zeros(Options.num_intervals,1), ...
                        'DxyMult', zeros(Options.num_intervals,1), ...
                        'scale', zeros(Options.num_intervals,1), ...
                        'edge_scale', -1);

octaves = repmat(proto,1,Options.num_octaves);
                 
%% Construct octaves
for i = 1:Options.num_octaves
    % Construct intervals in each octave
    for k = 1:Options.num_intervals
        % Calculate lobe length
        interval_size = s*2^(i-1) / (Options.num_intervals-2);
        octaves(i).scale(k) = s*2^(i-1) + interval_size*(k-1) - 0.5 ;  % SURF-style scale spacing with overlap
        
        % Compute mask lookups, using the custom filter parameters
        % Dyy mask
        octaves(i).dyy{k} = [surf_iiAreaLookupsC(0, 0, l2*octaves(i).scale(k), (2+2*l1)*octaves(i).scale(k), 1,     Options.integer_lookups); ...
                             surf_iiAreaLookupsC(0, 0, l2*octaves(i).scale(k), 2*octaves(i).scale(k),        -1-l1, Options.integer_lookups)];

        % Dxx mask
        octaves(i).dxx{k} = [surf_iiAreaLookupsC(0, 0, (2+2*l1)*octaves(i).scale(k), l2*octaves(i).scale(k), 1,     Options.integer_lookups); ...
                             surf_iiAreaLookupsC(0, 0, 2*octaves(i).scale(k),        l2*octaves(i).scale(k), -1-l1, Options.integer_lookups)];
        
        % Dxy mask
        octaves(i).dxy{k} = [surf_iiAreaLookupsC(-(l3+l4)*octaves(i).scale(k)/2, -(l3+l4)*octaves(i).scale(k)/2, l3*octaves(i).scale(k), l3*octaves(i).scale(k),  1, Options.integer_lookups); ...
                             surf_iiAreaLookupsC( (l3+l4)*octaves(i).scale(k)/2, -(l3+l4)*octaves(i).scale(k)/2, l3*octaves(i).scale(k), l3*octaves(i).scale(k), -1, Options.integer_lookups); ...
                             surf_iiAreaLookupsC(-(l3+l4)*octaves(i).scale(k)/2,  (l3+l4)*octaves(i).scale(k)/2, l3*octaves(i).scale(k), l3*octaves(i).scale(k), -1, Options.integer_lookups); ...
                             surf_iiAreaLookupsC( (l3+l4)*octaves(i).scale(k)/2,  (l3+l4)*octaves(i).scale(k)/2, l3*octaves(i).scale(k), l3*octaves(i).scale(k),  1, Options.integer_lookups)];
                         
        
        % Filter strength scaling factors
        % - scale by the area of the active regions of the filter, which
        % should be 1/octaves(i).scale(k)^2, with a constant factor
        % - constant factors are taken into account by rescaling
        % edge_scale, retaining the input relative weighting, but adapted
        % properly for the input filter dimensions
        % - all filter strengths are therefore scaled up by (2+2*l1)*l2, so
        % the threshold must be appropriately adapted
        octaves(i).DxxMult(k) = 1/(octaves(i).scale(k) * octaves(i).scale(k));
        octaves(i).DyyMult(k) = 1/(octaves(i).scale(k) * octaves(i).scale(k));
        octaves(i).DxyMult(k) = 1/(octaves(i).scale(k) * octaves(i).scale(k));
        octaves(i).edge_scale = edge_scale * ( (2+2*l1)*l2 / (4*l3*l3) )^2;
    end
    
    % Compute image mask border and step size
    octaves(i).border = ceil(octaves(i).scale(Options.num_intervals) * max([1+l1, l2/2, l3+l4/2]));
    octaves(i).step = Options.initial_step * (2^(i-1));
    
    % Store which octave this is
    octaves(i).number = i;
end

