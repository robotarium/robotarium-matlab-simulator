%% completeGL: $\mathbf{Z}^{+} \to \mathbf{Z}^{N \times N}$
% Returns a completely connected graph Laplacian 
%% Example Usage
%   L = completeGL(5); 
%% Implementation
function [ L ] = completeGL( n )
    L = n * eye(n) - ones(n,n);
end

