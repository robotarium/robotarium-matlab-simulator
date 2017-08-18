%% topological_neighbors $\mathbf{R}^{N \times N} \times \mathbf{Z}^{+} \to \mathbf{Z}^{+}$
% Returns the topological neighbors of a given agent

function [neighbors] = topological_neighbors(L, agent)
    
    N = size(L, 2); 
    
    assert(agent <= N && agent >= 1, 'Supplied agent (%i) must be between 1 and %i', agent, N);
    
    L_agent = L(agent, :);
    L_agent(agent) = 0;
    
    neighbors = find(L_agent ~= 0);
end

