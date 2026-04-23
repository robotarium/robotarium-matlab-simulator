% CONSENSUS_FEWER_ERRORS
% Demonstrates a submission-ready example of the consensus algorithm with
% a static, undirected cyclic graph topology. Same as consensus.m but
% with implementation considerations added: velocity thresholding to avoid
% actuator limit errors and a barrier certificate to prevent collisions 
% and boundary violations.
%
% Sean Wilson
% 07/2019

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 12;          % Number of robots
iterations = 2000; % Total simulation steps

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
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

% Backwards-motion-capable SI-to-unicycle mapping, more robust than
% create_si_to_uni_mapping for consensus where agents may need to reverse
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics();

% Barrier certificate prevents collisions and boundary violations
uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary();

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

    % ---------------------------------------------------------
    % Velocity thresholding
    % Caps speeds at 3/4 of max linear velocity to avoid
    % actuator limit errors
    % ---------------------------------------------------------
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4 * r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold * dxi(:, to_thresh) ./ norms(to_thresh);

    % ---------------------------------------------------------
    % Convert to unicycle dynamics and apply barrier certificate
    % ---------------------------------------------------------
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert_boundary(dxu, x);

    r.set_velocities(1:N, dxu);
    r.step();
end

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();