%% lineGL: $\mathbf{Z}^{+} \to \mathbf{Z}^{N \times N}$
% Returns a line graph Laplacian of size n x n 
%% Example Usage 
%   L = lineGL(5)
%% Implementation
function [ L ] = lineGL(n)
    L = 2*eye(n) - diag(ones(1,(n-1)), 1) - diag(ones(1, n-1), -1);
    L(1, 1) = 1; 
    L(n, n) = 1;
end

