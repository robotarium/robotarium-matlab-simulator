function L = completeGL(n)
% COMPLETEGL  Return the graph Laplacian of a complete graph on n nodes.
%
%   In a complete graph every node is connected to every other node, so
%   each node has degree n-1.  The Laplacian is L = n*I - 11^T.
%
%   Input
%   -----
%   n : number of nodes (positive integer)
%
%   Output
%   ------
%   L : nxn graph Laplacian
%
%   Example
%   -------
%       L = completeGL(5);

    L = n * eye(n) - ones(n);
end
