% FORMATION_CONTROL
% Demonstrates formation control utilizing edge tension energy with a
% static, undirected communication topology. Six robots converge to and
% maintain a regular hexagonal formation using a potential field law that
% drives each inter-agent distance toward a desired value.
%
% Paul Glotfelter, updated by Sean Wilson
% 07/2019

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 6;           % Number of robots
iterations = 3000; % Total simulation steps
formation_control_gain = 10; % Scales the edge tension potential gradient

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

initial_conditions = generate_random_poses(N, 'Width', 2, 'Height', 1, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

%% =========================================================
%  GRAPH TOPOLOGY
%  =========================================================

% Communication topology for the desired hexagonal formation.
% Robots 1–6 are arranged around the ring. Each robot is connected to its
% two immediate ring neighbours (6 edges) and to the diametrically opposite
% robot (3 long diagonals), giving 9 edges total — the minimum required
% for a rigid formation in 2D (2*N - 3 = 9).  Every node has degree 3.
%
%   Ring edges      : (1,2),(2,3),(3,4),(4,5),(5,6),(6,1)
%   Long diagonals  : (1,4),(2,5),(3,6)
%
L = [ 3 -1  0 -1  0 -1;
     -1  3 -1  0 -1  0;
      0 -1  3 -1  0 -1;
     -1  0 -1  3 -1  0;
      0 -1  0 -1  3 -1;
     -1  0 -1  0 -1  3];

%% =========================================================
%  FORMATION GEOMETRY
%  =========================================================

% Side length of the regular hexagon (distance between adjacent ring nodes).
d = 0.5;

% Distance between diametrically opposite nodes (long diagonal).
% For a regular hexagon: d_long = 2 * d.
d_long = 2 * d;

% Weight matrix specifying the desired distance for each connected pair.
% Zero entries indicate no edge between those agents.
%   d      : adjacent ring pairs
%   d_long : opposite pairs connected by long diagonals
weights = [0      d      0      d_long 0      d;
           d      0      d      0      d_long 0;
           0      d      0      d      0      d_long;
           d_long 0      d      0      d      0;
           0      d_long 0      d      0      d;
           d      0      d_long 0      d      0];

%% =========================================================
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

% Barrier certificate keeps robots from colliding or leaving the arena
si_barrier_cert = create_si_barrier_certificate_with_boundary();

% Maps single-integrator velocities to unicycle commands
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.5, 'AngularVelocityLimit', pi/2);

% Initialize single-integrator velocity vector
dx = zeros(2, N);

%% =========================================================
%  MAIN SIMULATION LOOP
%  =========================================================

for t = 0:iterations

    % Retrieve the most recent poses from the Robotarium.
    % The time delay is approximately 0.033 seconds.
    x = r.get_poses();

    % ---------------------------------------------------------
    % Formation control algorithm
    % Each agent applies an edge tension potential toward each
    % neighbor, driving the actual inter-agent distance toward
    % the desired value in weights(i,j)
    % ---------------------------------------------------------
    for i = 1:N
        dxi(:, i) = [0; 0];
        for j = topological_neighbors(L, i)
            dxi(:, i) = dxi(:, i) + ...
                formation_control_gain * (norm(x(1:2,i) - x(1:2,j))^2 - weights(i,j)^2) ...
                * (x(1:2,j) - x(1:2,i));
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
    dxi = si_barrier_cert(dxi, x);
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