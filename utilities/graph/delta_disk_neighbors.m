function neighbors = delta_disk_neighbors(poses, agent, delta)
% DELTA_DISK_NEIGHBORS  Return the indices of all agents within a given
% distance of a specified agent.
%
%   Computes the 2-norm distance from the agent to every other agent and
%   returns those whose distance is at most delta (the "delta-disk"
%   neighbourhood).  The agent itself is never included in the result.
%
%   Inputs
%   ------
%   poses : 2xN (or larger) matrix of agent positions; only rows 1-2 are used
%   agent : index of the query agent (1-based integer)
%   delta : neighbourhood radius (m)
%
%   Output
%   ------
%   neighbors : 1xK row vector of neighbour indices (may be empty)
%
%   Example
%   -------
%       neighbors = delta_disk_neighbors(poses, 1, 0.5);

    N = size(poses, 2);

    assert(agent >= 1 && agent <= N, ...
        'agent (%d) must be between 1 and %d.', agent, N);

    % Build index list of all agents except the query agent.
    others = [1:agent-1, agent+1:N];

    % Keep only those within the delta-disk.
    in_range  = vecnorm(poses(1:2, others) - poses(1:2, agent)) <= delta;
    neighbors = others(in_range);
end
