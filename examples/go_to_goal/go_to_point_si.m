% GO_TO_POINT_SI
% Drives N robots to randomized goal positions using a single-integrator
% proportional position controller, mapped to unicycle inputs via the
% SI-to-uni dynamics, with a barrier certificate for safety.
%
% Unlike the pose examples (go_to_pose_CLF, go_to_pose_hybrid), this
% example targets position only — final heading is not controlled.
%
% See also: create_si_position_controller, create_si_barrier_certificate_with_boundary

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 6; % Number of robots

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

initial_positions = generate_random_poses(N, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% =========================================================
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

% Barrier certificate keeps robots from colliding or leaving the arena
si_barrier_certificate = create_si_barrier_certificate_with_boundary();

% SI position controller drives robots to a 2D goal position [x; y]
si_position_controller = create_si_position_controller();

% SI-to-uni mapping converts single-integrator velocities to [v; omega]
si_to_uni = create_si_to_uni_dynamics();

% Convergence threshold: a robot is considered done when it is within
% this distance of its goal
position_error = 0.05;

% Convergence checker — returns (done, 1xN logical idxs)
at_position = create_at_position('PositionError', position_error);

%% =========================================================
%  GOAL INITIALIZATION
%  =========================================================

% Randomized goal positions within the arena, kept away from the boundary
% by one robot diameter on each side.
% Goals are 2xN [x; y] — no heading target for a point controller.
arena_width  = r.boundaries(2) - r.boundaries(1) - 3*r.robot_diameter;
arena_height = r.boundaries(4) - r.boundaries(3) - 3*r.robot_diameter;

goal_points = generate_random_poses(N, ...
    'Width',   arena_width, ...
    'Height',  arena_height, ...
    'Spacing', 0.5);
goal_points = goal_points(1:2, :);  % drop the theta row — only position needed

% Get initial poses and take one step before entering the loop
x = r.get_poses();
r.step();

%% =========================================================
%  MAIN LOOP
%  =========================================================

% Track which robots have already been reported as arrived so each robot
% only prints once even if it briefly leaves and re-enters the threshold
already_reported = false(1, N);

while ~at_position(x, goal_points)

    x = r.get_poses();

    % Compute SI velocities toward goal positions
    dxi = si_position_controller(x(1:2,:), goal_points);

    % Apply SI barrier certificate to prevent collisions and boundary violations
    dxi = si_barrier_certificate(dxi, x);

    % Map SI velocities to unicycle inputs
    dxu = si_to_uni(dxi, x);

    r.set_velocities(1:N, dxu);
    r.step();

    % Print a message the first time each robot reaches its goal position
    [~, converged]  = at_position(x, goal_points);
    newly_arrived   = converged & ~already_reported;
    for i = find(newly_arrived)
        fprintf('Robot %d has reached its goal position.\n', i);
    end
    already_reported = already_reported | converged; % Remembered reported or newly converged check.
end

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();