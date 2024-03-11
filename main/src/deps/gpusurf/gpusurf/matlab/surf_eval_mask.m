function val = surf_eval_mask( iimg, ix, iy, mask )
% SURF_EVAL_MASK Evaluates a mask on an integral image, emulating the GPU
%                tex2D() function calls.
%
% val = surf_eval_mask( iimg, ix, iy, mask )
%
% INPUTS:
% iimg  - integral image
% ix,iy - the coordinates of the center of the mask (ix,iy)
% mask  - Nx3 array containing rows of: [multiplier, x_offset, y_offset]
%
% OUTPUT:
% val - the evaluated value of the mask
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

%% Evaluate the mask
val = 0;
[M N] = size(iimg);

for i = 1:size(mask,1)
    % Compute the lookup coordinates
    % - subtract 0.5 to match the GPU tex2D() call
    y = mask(i,3) + iy - 0.5;
    x = mask(i,2) + ix - 0.5;
    
    % The direct method is: v = iimg(y,x)
    % (but bilinear interpolation emulates what we would get on the GPU)
    % Extract the integer and remainder parts of the lookup.
    fix = floor(x);
    dix = x - fix;
    
    fiy = floor(y);
    diy = y - fiy;
    
    % Limit the ranges to emulate the GPU's boundary condition.
    fx = min(N,max(1,fix));
    cx = min(N,max(1,fix+1));
    
    fy = min(M,max(1,fiy));
    cy = min(M,max(1,fiy+1));
    
    % Compute bilinear interpolation
    v =   (1-dix)*(1-diy)*iimg(fy,fx) + dix * (1-diy) * iimg(fy,cx)...
        + (1-dix)*  diy  *iimg(cy,fx) + dix *   diy   * iimg(cy,cx);
        
    % Add the scaled area to the mask response
    val = val + v * mask(i,1);
end

