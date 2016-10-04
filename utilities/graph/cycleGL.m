%% cycleGL: $\mathbf{Z}^{+} \to \mathbf{Z}^{N \times N}$
% Returns a cycle graph Laplacian 
%% Example Usage 
%   L = cycleGL(4)
%% Implementation
function [ L ] = cycleGL( n )
    L = 2*eye(n) - diag(ones(1,(n-1)), 1) - diag(ones(1,(n-1)), -1);
    L(n, 1) = -1;
    L(1, n) = -1;
end

