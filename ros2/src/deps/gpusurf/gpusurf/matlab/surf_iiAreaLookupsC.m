function lookups = surf_iiAreaLookupsC(x, y, w, h, scaleFactor, INTEGER_LOOKUPS)
% SURF_IIAREALOOKUPSC Computes the integral image lookups required for an 
%                     area computation specified by the center position.
%
% lookups = surf_iiAreaLookupsC(x, y, w, h, scaleFactor, INTEGER_LOOKUPS)
%
% INPUTS:
% x,y             - the center pixel coordinates of the area computation
% w               - the width of the rectangular area sought (x)
% h               - the height of the rectangular area sought (y)
% scaleFactor     - the scaling factor to be applied to the area value
% INTEGER_LOOKUPS - flag designating whether or not the lookups should be
%                   rounded to the nearest integer or not
%
% OUTPUT:
% lookups - a 4x3 array consisting of values formatted correctly for input
%           into the surf_eval_mask() function: [multiplier, x_offset, y_offset]
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

%% Input check
if (nargin < 5)
    scaleFactor = 1;
end

if (nargin < 6)
    INTEGER_LOOKUPS = false;
end

%% Compute the lookup positions
% A (x1,y1) ------ B (x2,y1)
% |                |            area = A-B-C+D
% C (x1,y2) ------ D (x2,y2)
%
x1 = x - w/2;
y1 = y - h/2;
x2 = x + w/2;
y2 = y + h/2;

if (INTEGER_LOOKUPS)
    x1 = round(x1);
    y1 = round(y1);
    x2 = round(x2);
    y2 = round(y2);
end

lookups = [ scaleFactor, x1, y1; ...
           -scaleFactor, x2, y1; ...
           -scaleFactor, x1, y2; ...
            scaleFactor, x2, y2];

