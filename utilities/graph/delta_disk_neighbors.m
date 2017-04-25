%% delta_disk_neighbors $\mathbf{R}^{2 \times N} \times \mathbf{Z}^{+} \times \mathbf{R} \to \mathbf{Z}^{+}$
% Returns the agents within the 2-norm of the supplied agent
%% Example Usage 
%   neighbors = delta_disk_neighbors(robot_poses, 1, 0.5);

function [ neighbors ] = delta_disk_neighbors(poses, agent, delta)
    
    N = size(poses, 2);
    agents = 1:N;
    agents(agent) = [];
    
    assert(agent<=N && agent>=1, 'Supplied agent (%i) must be between 1 and %i', agent, N);

    within_distance = arrayfun(@(x) norm(poses(1:2, x) - poses(1:2, agent)) <= delta, agents);
    neighbors = agents(within_distance);
end

