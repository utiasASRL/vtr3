function Options = surf_init_options( Options )
% SURF_INIT_OPTIONS Returns a Options struct with the uninitialized fields 
%                   initialized with their default parameters.
%
% options = surf_init_options( Options )
%
% SurfOptions is a struct, comprising of the following fields:
%
% Filter parameters:
%   s           - the initial scale (in pixels) of the box filters
%   l1,l2,l3,l4 - the filter dimension parameters (see documentation for
%                 illustration)
%   edge_scale  - the relative weighting applied between the Dxx/Dyy and 
%                 Dxy box filter responses
%
% Interest point thresholding:
%   threshold    - a threshold on the interest point strengths
%   num_features - the target number of features to return
%
% Fast Hessian configuration:
%   num_octaves   - number of octaves
%   num_intervals - number of intervals per octave
%   initial_step  - initial step size
%
% Algorithm configuration:
%   integer_lookups           - whether to do integer-rounded integral
%                               image lookups (like the standard SURF), or
%                               allow subpixel indexing (like the GPU)
%   do_subpixel_interpolation - a flag indicating whether or not to do 
%                               subpixel/subscale keypoint interpolation
%   zero_indexed              - if true, the returned keypoints are zero 
%                               indexed. Otherwise, they are returned one 
%                               indexed (like how Matlab does it)
%   max_feats_per_octave      - maximum number of maxima allowed to be
%                               found in each octave
%   ori_window_size           - dominant orientation computation sliding
%                               window size
%   ori_restrict_to_circle    - a flag for whether to restrict the lattice 
%                               samples in the orientation computation to
%                               inside the circle (like SURF), or to use
%                               all of the points (like the GPU does for
%                               speedup)
%   fast_orientation          - a faster orientation calculation (no
%                               sliding window used)
%   upright_descriptor        - whether or not to use an upright descriptor
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
if (nargin < 1)
    Options = struct;
end

%% Filter parameters
% Default parameters are such that the first octave matches that of SURF
if (~isfield(Options,'s'))
    Options.s = 2;
end

if (~isfield(Options,'l1'))
    Options.l1 = 3/1.5;
end

if (~isfield(Options,'l2'))
    Options.l2 = 5/1.5;
end

if (~isfield(Options,'l3'))
    Options.l3 = 3/1.5;
end

if (~isfield(Options,'l4'))
    Options.l4 = 1/1.5;
end

if (~isfield(Options,'edge_scale'))
    Options.edge_scale = 0.81;
end

%% Interest point thresholding
if (~isfield(Options,'threshold'))
    Options.threshold = 0.1;
end

if (~isfield(Options,'num_features'))
    Options.num_features = Inf;
end

%% Fast Hessian configuration
if (~isfield(Options,'num_octaves'))
    Options.num_octaves = 4;
end

if (~isfield(Options,'num_intervals'))
    Options.num_intervals = 4;
end

if (~isfield(Options,'initial_step'))
    Options.initial_step = 1;
end

%% Algorithm configuration
if (~isfield(Options,'integer_lookups'))
    Options.integer_lookups = false;
end

if (~isfield(Options,'do_subpixel_interpolation'))
    Options.do_subpixel_interpolation = true;
end

if (~isfield(Options,'zero_indexed'))
    Options.zero_indexed = false;
end

if (~isfield(Options,'max_feats_per_octave'))
    Options.max_feats_per_octave = 4096;
end

if (~isfield(Options,'ori_window_size'))
    Options.ori_window_size = pi/3;
end

if (~isfield(Options,'ori_restrict_to_circle'))
    Options.ori_restrict_to_circle = false;
end

if (~isfield(Options,'fast_orientation'))
    Options.fast_orientation = false;
end

if (~isfield(Options,'upright_descriptor'))
    Options.upright_descriptor = false;
end

end

