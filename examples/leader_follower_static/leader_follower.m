% LEADER_FOLLOWER
% Demonstrates leader-follower control with a static, undirected topology.
% One leader robot travels between four waypoints while N-1 follower robots
% maintain a formation relative to it using a graph Laplacian-based
% formation control law.
%
% Paul Glotfelter, edited by Sean Wilson
% 07/2019

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 4;           % Number of robots (1 leader + N-1 followers)
iterations = 5000; % Total simulation steps

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

initial_positions = generate_random_poses(N, 'Width', 1, 'Height', 1, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% =========================================================
%  GRAPH LAPLACIAN AND FORMATION SETUP
%  =========================================================

% Build graph Laplacian: followers form a complete graph among themselves,
% with robot 2 additionally connected to the leader (robot 1)
followers = -completeGL(N-1);
L = zeros(N, N);
L(2:N, 2:N) = followers;
L(2, 2) = L(2, 2) + 1;
L(2, 1) = -1;

formation_control_gain = 10; % Gain for formation control law
desired_distance = 0.2;      % Desired inter-robot distance (meters)

%% =========================================================
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

% Maps single-integrator velocities to unicycle commands
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);

% Barrier certificate keeps robots from colliding or leaving the arena
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();

% Position controller drives the leader toward each waypoint
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.1);

%% =========================================================
%  WAYPOINT INITIALIZATION
%  =========================================================

% Leader cycles through these four corner waypoints in order
waypoints = [-1 0.8; -1 -0.8; 1 -0.8; 1 0.8]';
close_enough = 0.05; % Distance threshold to consider a waypoint reached

% Leader starts targeting waypoint 1
state = 1;

% Initialize single-integrator velocity vector
dxi = zeros(2, N);

%% =========================================================
%  MAIN SIMULATION LOOP
%  =========================================================

for t = 1:iterations

    % Retrieve the most recent poses from the Robotarium.
    % The time delay is approximately 0.033 seconds.
    x = r.get_poses();

    % ---------------------------------------------------------
    % Formation control for followers
    % Each follower moves to maintain desired_distance from
    % its graph neighbors using a potential field law
    % ---------------------------------------------------------
    for i = 2:N
        dxi(:, i) = [0; 0];
        neighbors = topological_neighbors(L, i);
        for j = neighbors
            dxi(:, i) = dxi(:, i) + ...
                formation_control_gain * (norm(x(1:2,j) - x(1:2,i))^2 - desired_distance^2) * (x(1:2,j) - x(1:2,i));
        end
    end

    % ---------------------------------------------------------
    % Leader waypoint tracking
    % Cycles through waypoints in order; advances state when
    % the leader gets within close_enough of the current waypoint
    % ---------------------------------------------------------
    waypoint = waypoints(:, state);
    dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
    if norm(x(1:2, 1) - waypoint) < close_enough
        state = mod(state, length(waypoints)) + 1;
    end

    % ---------------------------------------------------------
    % Velocity thresholding for followers
    % Caps follower speeds at 3/4 of max linear velocity to
    % avoid actuator limit errors. Leader is excluded (index 1).
    % ---------------------------------------------------------
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4 * r.max_linear_velocity;
    to_thresh = norms > threshold;
    to_thresh(1) = 0;
    dxi(:, to_thresh) = threshold * dxi(:, to_thresh) ./ norms(to_thresh);

    % ---------------------------------------------------------
    % Convert to unicycle dynamics and apply barrier certificate
    % ---------------------------------------------------------
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);

    r.set_velocities(1:N, dxu);
    r.step();
end

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();