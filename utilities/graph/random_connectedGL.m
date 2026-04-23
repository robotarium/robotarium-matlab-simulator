function L = random_connectedGL(v, e)
% RANDOM_CONNECTEDGL  Return the graph Laplacian of a random connected graph.
%
%   Generates a guaranteed-connected graph on v nodes using a random
%   spanning tree (Prüfer-style: each new node i connects to a uniformly
%   random existing node), then adds up to e extra edges chosen uniformly
%   at random from the remaining candidate pairs.  The total edge count is
%   therefore between (v-1) and (v-1)+e.
%
%   Inputs
%   ------
%   v : number of nodes (positive integer)
%   e : number of extra edges to add beyond the spanning tree (non-negative integer)
%
%   Output
%   ------
%   L : vxv graph Laplacian
%
%   Example
%   -------
%       L = random_connectedGL(6, 4);

    L = zeros(v);

    % ── Build a random spanning tree ─────────────────────────────────────
    % Add each node i (2..v) with an edge to a uniformly random predecessor.
    % This guarantees connectivity with exactly v-1 edges.
    for i = 2:v
        j = randi(i - 1);
        L(i, j) = -1;   L(j, i) = -1;
        L(i, i) = L(i, i) + 1;
        L(j, j) = L(j, j) + 1;
    end

    % ── Add extra random edges ────────────────────────────────────────────
    % Candidate pairs are upper-triangular entries that are not yet edges
    % (L entry == 0) and are not self-loops (off-diagonal).
    % triu(...,1) restricts to strictly upper triangle, excluding diagonal.
    candidates = find(triu(L == 0, 1));

    numEdges = min(e, numel(candidates));
    if numEdges <= 0
        return
    end

    chosen = candidates(randperm(numel(candidates), numEdges));
    for k = 1:numEdges
        [i, j] = ind2sub([v, v], chosen(k));
        L(i, j) = -1;   L(j, i) = -1;
        L(i, i) = L(i, i) + 1;
        L(j, j) = L(j, j) + 1;
    end
end
