function neighbors = topological_neighbors(L, agent)
% TOPOLOGICAL_NEIGHBORS  Return the indices of an agent's graph neighbours.
%
%   Reads connectivity directly from the graph Laplacian: any off-diagonal
%   entry in the agent's row that is non-zero indicates an edge, and
%   therefore a neighbour.
%
%   Inputs
%   ------
%   L     : NxN graph Laplacian
%   agent : index of the query agent (1-based integer)
%
%   Output
%   ------
%   neighbors : 1xK row vector of neighbour indices (may be empty)
%
%   Example
%   -------
%       neighbors = topological_neighbors(L, 3);

    N = size(L, 1);

    assert(agent >= 1 && agent <= N, ...
        'agent (%d) must be between 1 and %d.', agent, N);

    % Extract the agent's row and zero out the diagonal before searching,
    % since a non-zero diagonal encodes degree, not a self-edge.
    row        = L(agent, :);
    row(agent) = 0;

    neighbors = find(row ~= 0);
end
