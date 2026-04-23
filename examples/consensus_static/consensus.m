% CONSENSUS
% Demonstrates a theoretical example of the consensus algorithm with a
% static, undirected cyclic graph topology. All agents converge to the
% same position using only local neighborhood information.
%
% NOTE: This script lacks implementation considerations such as obstacle
% avoidance, boundary enforcement, and velocity thresholding. If submitted
% to the Robotarium it will be rejected. See consensus_fewer_errors.m for
% a submission-ready version.
%
% Sean Wilson
% 07/2019

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 12;          % Number of robots
iterations = 1000; % Total simulation steps

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%% =========================================================
%  GRAPH TOPOLOGY
%  =========================================================

% Cyclic graph Laplacian — any connected graph will yield consensus
L = cycleGL(N);

%% =========================================================
%  CONTROLLER SETUP
%  =========================================================

% Diffeomorphism mapping between single-integrator and unicycle dynamics
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping();

% Initialize single-integrator velocity vector
dxi = zeros(2, N);

%% =========================================================
%  MAIN SIMULATION LOOP
%  =========================================================

for t = 1:iterations

    % Retrieve the most recent poses from the Robotarium.
    % The time delay is approximately 0.033 seconds.
    x = r.get_poses();

    % Convert unicycle states to single-integrator states
    xi = uni_to_si_states(x);

    % ---------------------------------------------------------
    % Consensus algorithm
    % Each agent sums the relative positions of its graph
    % neighbors to drive all agents toward agreement
    % ---------------------------------------------------------
    for i = 1:N
        dxi(:, i) = [0; 0];
        neighbors = topological_neighbors(L, i);
        for j = neighbors
            dxi(:, i) = dxi(:, i) + (xi(:, j) - xi(:, i));
        end
    end

    % Convert single-integrator to unicycle velocities
    dxu = si_to_uni_dyn(dxi, x);

    r.set_velocities(1:N, dxu);
    r.step();
end

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();