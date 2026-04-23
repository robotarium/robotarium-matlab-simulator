function L = cycleGL(n)
% CYCLEGL  Return the graph Laplacian of a cycle graph on n nodes.
%
%   In a cycle graph every node has exactly two neighbours (the nodes
%   immediately before and after it in the ring), so every diagonal entry
%   is 2.  The two wrap-around edges between nodes 1 and n are added
%   explicitly after the tridiagonal base is formed.
%
%   Input
%   -----
%   n : number of nodes (positive integer, n >= 3)
%
%   Output
%   ------
%   L : nxn graph Laplacian
%
%   Example
%   -------
%       L = cycleGL(6);

    % Tridiagonal base: accounts for all internal edges.
    L = 2*eye(n) - diag(ones(1, n-1), 1) - diag(ones(1, n-1), -1);

    % Close the ring by connecting node n back to node 1.
    L(1, n) = -1;
    L(n, 1) = -1;
end
