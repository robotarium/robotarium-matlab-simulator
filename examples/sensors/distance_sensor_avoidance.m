%% Robotarium Obstacle Avoidance Example
% This script demonstrates a robot navigating to a goal while dynamically
% mapping and avoiding obstacles using distance sensors and a control
% barrier certificate.

% --- Configuration & Environment Setup ---
AVOID_DISTANCE = 0.15; % Safety buffer around obstacles
N = 5;                 % Number of robots

% Define static boundary obstacles [x1, y1; x2, y2]
obstacles = zeros(4, 2, 2);
obstacles(1, :, :) = [-1.6, -1.0; 1.6, -1.0];
obstacles(2, :, :) = [-1.6,  1.0; 1.6,  1.0];
obstacles(3, :, :) = [-1.6, -1.6; -1.6, 1.0];
obstacles(4, :, :) = [ 1.6, -1.6;  1.6, 1.0];

% Set robot starting poses: [x; y; theta]
initial_conditions = [
    -1.2, -0.6, -0.0, -0.0, 0.6;
    0.0, 0.0, 0.6, -0.6, 0.0;
    0.0, 0.0, 0.0, 0.0, 0.0
];

% Initialize the Robotarium simulator
r = Robotarium("NumberOfRobots", 5, "ShowFigure", true, ...
               "InitialConditions", initial_conditions, "Obstacles", obstacles, ...
               "UseDistanceSensors", true, "ShowDistanceEndpoints", false, ...
               "ShowDistanceRays", false, "ShowObstacles", true, ...
               "SkipInitialization", true);

% Single-integrator position controller drives robots toward goal positions
controller = create_si_position_controller();
% Maps single-integrator velocities to unicycle commands
si_to_uni_dynamics = create_si_to_uni_dynamics();
% Determine when the robot is close to the goal
at_position = create_at_position("PositionError", AVOID_DISTANCE);

% Define goals (Robot 1 moves to [1.2, 0], others stay put)
goals = initial_conditions;
goals(1:2, 1) = [1.2, 0.0];

% Visual Feedback: Draw the goal region
goal_center = goals(1:2, 1);
radius = AVOID_DISTANCE - 0.02;
pos = [goal_center(1)-radius, goal_center(2)-radius, 2*radius, 2*radius];
goal_patch = rectangle("Position", pos, ...
    "Curvature", [1 1], ...
    "FaceColor", [0 0 1 0.5], ...
    "EdgeColor", "none");
goal_label = text(goals(1, 1), goals(2, 1), "Goal", ...
    "FontSize", 12, ...
    "HorizontalAlignment", "center", ...
    "VerticalAlignment", "middle");

% Data structures for dynamic mapping
hitpoints = [];          % Temporary point cloud from sensors
hitpoint_counts = [];   % Observation frequency to filter noise
obstacle_locations = []; % Confirmed obstacle centers

x = r.get_poses();
r.step();

%% --- Main Simulation Loop ---
while ~at_position(x(1:2, :), goals(1:2, :))
    x = r.get_poses();

    % Process Sensor Data
    distances = r.get_distances();
    endpoints = r.get_distance_endpoints();

    for i = 1:size(distances, 1)
        if distances(i, 1) > 0.0 && distances(i, 1) < 1.0
            % Check if a sensor ray hit something within range (1.0m)
            endpoint = endpoints(:, i, 1);
            new_obstacle = false;
            new_hitpoint = true;

            % Clustering logic: Check if this hit aligns with an existing point
            for j = 1:size(hitpoints, 1)
                if norm(endpoint - hitpoints(j, :)') < AVOID_DISTANCE
                    new_hitpoint = false;
                    hitpoint_counts(j) = hitpoint_counts(j) + 1;

                    % If we hit the same spot 5 times, confirm it as an obstacle
                    if hitpoint_counts(j) == 10
                        new_obstacle = true;
                        break;
                    end
                end
            end

            % Record new sensor hit
            if new_hitpoint
                hitpoints = [hitpoints; endpoint'];
                hitpoint_counts = [hitpoint_counts; 1];
            end

            % If a new obstacle is confirmed, visualize it
            if new_obstacle
                obstacle_locations = [obstacle_locations; hitpoints(j, :)];
                pos = [endpoint(1)-radius, endpoint(2)-radius, 2*radius, 2*radius];
                obstacle_patch = rectangle('Position', pos, ...
                    'Curvature', [1 1], 'FaceColor', [1 0 0 0.5], 'EdgeColor', 'none');
            end
        end
    end

    % Calculate desired Single-Integrator (SI) velocity
    dxi = controller(x(1:2, :), goals(1:2, :));
    % Apply obstacle avoidance barrier certificate to the first robot
    dxi(:, 1) = obstacle_barrier_certificate(dxi(:, 1), x(:, 1), obstacle_locations, 0.15, AVOID_DISTANCE);

    % Convert SI to Unicycle dynamics and move
    dxu = si_to_uni_dynamics(dxi, x);
    r.set_velocities(1:N, dxu);
    r.step();
end

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();
%% --- Helper Functions ---

function dxi = obstacle_barrier_certificate(dx, x, centers, magnitude_limit, avoid_distance)
    num_obstacles = size(centers, 1);
    num_constraints = num_obstacles + 8;
    A = zeros(num_constraints, 2);
    b = zeros(num_constraints, 1);

    % Apply obstacle constraints
    for i = 1:num_obstacles
        diff = x(1:2)' - centers(i, :);
        h = dot(diff, diff) - avoid_distance^2;

        A(i, :) = -2 * diff;
        b(i) = 0.1 * h;
    end

    % Apply Velocity Actuator Constraints (octagonal approximation of speed limits)
    constraint = num_obstacles + 1;
    % vx <= magnitude_limit
    A(constraint, :) = [1, 0];
    b(constraint) = magnitude_limit * cos(pi / 8);
    constraint = constraint + 1;

    % 1/sqrt(2) * (vx + vy) <= magnitude_limit
    A(constraint, :) = [1/sqrt(2), 1/sqrt(2)];
    b(constraint) = magnitude_limit * cos(pi / 8);
    constraint = constraint + 1;

    % vy <= magnitude_limit
    A(constraint, :) = [0, 1];
    b(constraint) = magnitude_limit * cos(pi / 8);
    constraint = constraint + 1;

    % 1/sqrt(2) * (-vx + vy) <= magnitude_limit
    A(constraint, :) = [-1/sqrt(2), 1/sqrt(2)];
    b(constraint) = magnitude_limit * cos(pi / 8);
    constraint = constraint + 1;

    % -vx <= magnitude_limit
    A(constraint, :) = [-1, 0];
    b(constraint) = magnitude_limit * cos(pi / 8);
    constraint = constraint + 1;

    % 1/sqrt(2) * (-vx - vy) <= magnitude_limit
    A(constraint, :) = [-1/sqrt(2), -1/sqrt(2)];
    b(constraint) = magnitude_limit * cos(pi / 8);
    constraint = constraint + 1;

    % -vy <= magnitude_limit
    A(constraint, :) = [0, -1];
    b(constraint) = magnitude_limit * cos(pi / 8);
    constraint = constraint + 1;

    % 1/sqrt(2) * (vx - vy) <= magnitude_limit
    A(constraint, :) = [1/sqrt(2), -1/sqrt(2)];
    b(constraint) = magnitude_limit * cos(pi / 8);

    vhat = dx(:);
    [vnew, ~, exitflag] = quadprog(sparse(2*eye(2)), -2*vhat, A, b, [], [], [], [], [], optimoptions(@quadprog, 'Display', 'off'));

    if exitflag ~= 1
        warning("obstacle_barrier_certificate:QPFailed", "obstacle_barrier_certificate: QP failed (exitflag %d). Commanding zero velocities.", exitflag);
        dxi = zeros(2, 1);
    else
        dxi = vnew(:);
    end
end