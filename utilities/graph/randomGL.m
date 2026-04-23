function L = randomGL(v, e)
% RANDOMGL  Return the graph Laplacian of a random graph on v nodes with
% at most e edges.
%
%   Selects up to e edges uniformly at random from all possible undirected
%   pairs (no self-loops, no multi-edges).  The resulting graph may be
%   disconnected.  For a guaranteed-connected random graph use
%   random_connectedGL instead.
%
%   Inputs
%   ------
%   v : number of nodes (positive integer)
%   e : desired number of edges; capped at v*(v-1)/2 if larger
%
%   Output
%   ------
%   L : vxv graph Laplacian
%
%   Example
%   -------
%       L = randomGL(5, 3);

    L = zeros(v);

    % ── Enumerate candidate edges ─────────────────────────────────────────
    % All strictly upper-triangular indices correspond to unique undirected
    % pairs — no self-loops, no duplicates.
    candidates = find(triu(ones(v), 1));

    % ── Sample edges uniformly at random ─────────────────────────────────
    numEdges = min(e, numel(candidates));
    chosen   = candidates(randperm(numel(candidates), numEdges));

    for k = 1:numEdges
        [i, j] = ind2sub([v, v], chosen(k));
        L(i, j) = -1;   L(j, i) = -1;
        L(i, i) = L(i, i) + 1;
        L(j, j) = L(j, j) + 1;
    end
end
