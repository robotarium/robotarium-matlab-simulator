% GO_TO_POSE_CLF
% Drives N robots to randomized goal poses (x, y, theta) using a CLF-based
% unicycle pose controller and barrier certificate. Unlike go_to_pose.m
% which uses a waypoint controller, this example uses the CLF controller
% which directly minimizes a Lyapunov function over the full pose error,
% so robots converge to both position and orientation simultaneously rather
% than sequentially.
%
% See also: create_pose_controller_CLF, create_uni_barrier_certificate_with_boundary

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
unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();

% CLF controller drives robots to a full pose [x; y; theta]
pose_controller = create_pose_controller_clf();

% Convergence thresholds: a robot is considered done when its position
% error and heading error fall below these values
position_error  = 0.05;
rotation_error  = 0.2;

% Convergence checker — returns (done, 1xN logical idxs)
at_pose = create_at_pose('PositionError', position_error, 'RotationError', rotation_error);

%% =========================================================
%  GOAL INITIALIZATION
%  =========================================================

% Randomized goal poses within the arena, kept away from the boundary
% by one robot diameter on each side
arena_width  = r.boundaries(2) - r.boundaries(1) - 4*r.robot_diameter;
arena_height = r.boundaries(4) - r.boundaries(3) - 4*r.robot_diameter;

goal_poses = generate_random_poses(N, ...
    'Width',   arena_width, ...
    'Height',  arena_height, ...
    'Spacing', 0.5);

% Get initial poses and take one step before entering the loop
x = r.get_poses();
r.step();

%% =========================================================
%  MAIN LOOP
%  =========================================================

% Track which robots have already been reported as arrived so each robot
% only prints once even if it briefly leaves and re-enters the threshold
already_reported = false(1, N);

while ~at_pose(x, goal_poses)

    x = r.get_poses();

    % Compute unicycle velocities toward goal poses using the CLF law
    dxu = pose_controller(x, goal_poses);

    % Apply barrier certificate to prevent collisions and boundary violations
    dxu = unicycle_barrier_certificate(dxu, x);

    r.set_velocities(1:N, dxu);
    r.step();

    % Print a message the first time each robot reaches its goal position
    [~, converged]  = at_pose(x, goal_poses);
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
