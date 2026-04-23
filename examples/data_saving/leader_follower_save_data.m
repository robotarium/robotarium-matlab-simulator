% LEADER_FOLLOWER_SAVE_DATA
% Same script as leader_follower_with_plotting.m but with additional data
% saving. Two datasets are recorded and saved at the end of the experiment:
%
%   DistanceData.mat  — inter-robot distances for each connected pair plus
%                       elapsed time, sampled every iteration
%   GoalData.mat      — leader-to-goal distance and elapsed time recorded
%                       each iteration the leader is within close_enough
%                       of the current waypoint
%
% Sean Wilson
% 07/2019

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 4;           % Number of robots (1 leader + N-1 followers)
iterations = 5000; % Total simulation steps

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

initial_positions = generate_random_poses(N, 'Width', 1, 'Height', 1, 'Spacing', 0.3);
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

formation_control_gain = 10;  % Gain for formation control law
desired_distance = 0.2;       % Desired inter-robot distance (meters)

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
close_enough = 0.03; % Distance threshold to consider a waypoint reached

% Leader starts targeting waypoint 1
state = 1;

% Initialize single-integrator velocity vector
dxi = zeros(2, N);

%% =========================================================
%  PLOT INITIALIZATION
%  =========================================================

% One color per robot. Fixed string so colors are deterministic.
CM = ['k', 'b', 'r', 'g'];

% Scale marker and font sizes to the figure window so they look correct
% both in simulation and when deployed on the physical Robotarium
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 5;

% Draw waypoint goal markers and labels
g = gobjects(1, length(waypoints));
goal_labels = cell(1, length(waypoints));
for i = 1:length(waypoints)
    g(i) = plot(waypoints(1,i), waypoints(2,i), ...
        's', 'MarkerSize', marker_size_goal, 'LineWidth', line_width, 'Color', CM(i));
    goal_labels{i} = text(waypoints(1,i)-0.05, waypoints(2,i), ...
        sprintf('G%d', i), 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'k');
end

% Get initial poses so graph edge lines start in the right place.
% Must be called once before the first r.step().
x = r.get_poses();

% Draw follower-follower graph edges (blue).
% L == 1 gives directed edges; take only the first half to avoid
% drawing each undirected edge twice.
[rows, cols] = find(L == 1);
lf = gobjects(1, length(rows)/2+1);
for k = 1:length(rows)/2+1
    lf(k) = line([x(1,rows(k)), x(1,cols(k))], [x(2,rows(k)), x(2,cols(k))], ...
        'LineWidth', line_width, 'Color', 'b');
end

% Draw leader-follower graph edge (red).
% Only the connection between robot 1 and robot 2 is shown.
ll = line([x(1,1), x(1,2)], [x(2,1), x(2,2)], 'LineWidth', line_width, 'Color', 'r');

% Draw follower robot labels.
% Labels are placed off-screen initially and repositioned each step.
follower_caption = cell(1, N-1);
follower_labels = cell(1, N-1);
for j = 1:N-1
    follower_caption{j} = sprintf('Follower Robot %d', j);
    follower_labels{j} = text(500, 500, follower_caption{j}, 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'k');
end

% Draw leader robot label
leader_label = text(500, 500, 'Leader Robot', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');

%% =========================================================
%  DATA SAVING SETUP
%  =========================================================

% Row layout for robot_distance:
%   Row 1   — leader to robot 2 distance
%   Rows 2-4 — follower-follower pair distances (from graph edges)
%   Row 5   — elapsed time (seconds)
robot_distance = zeros(5, iterations);

% goal_distance cannot be preallocated since the number of times the
% leader reaches a waypoint is not known in advance. Columns are appended
% as waypoints are reached. Row layout: [distance; elapsed_time]
goal_distance = [];

start_time = tic;

r.step();

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

    % ---------------------------------------------------------
    % Update plot handles
    % ---------------------------------------------------------

    % Follower labels track their robot positions
    for q = 1:N-1
        follower_labels{q}.Position = x(1:2, q+1) + [-0.15; 0.15];
    end

    % Follower-follower graph edges
    for m = 1:length(rows)/2+1
        lf(m).XData = [x(1,rows(m)), x(1,cols(m))];
        lf(m).YData = [x(2,rows(m)), x(2,cols(m))];
    end

    % Leader label and leader-follower edge
    leader_label.Position = x(1:2, 1) + [-0.15; 0.15];
    ll.XData = [x(1,1), x(1,2)];
    ll.YData = [x(2,1), x(2,2)];

    % Resize markers and fonts in case the figure window was resized.
    % This is a no-op when deployed on the Robotarium since the figure
    % window is fixed size.
    new_goal_size = num2cell(ones(1, length(waypoints)) * determine_marker_size(r, 0.20));
    [g.MarkerSize] = new_goal_size{:};
    font_size = determine_font_size(r, 0.05);
    leader_label.FontSize = font_size;
    for n = 1:N
        % Font must be updated in a loop for MATLAB conversion reasons.
        % Unnecessary when submitting as the figure window does not resize.
        follower_labels{n}.FontSize = font_size;
        goal_labels{n}.FontSize = font_size;
    end

    % ---------------------------------------------------------
    % Record data
    % ---------------------------------------------------------

    % Inter-robot distances for each connected pair
    robot_distance(1, t) = norm(x(1:2,1) - x(1:2,2), 2);
    for b = 1:length(rows)/2+1
        robot_distance(b+1, t) = norm(x(1:2,rows(b)) - x(1:2,cols(b)), 2);
    end
    robot_distance(5, t) = toc(start_time);

    % Record leader-to-goal distance whenever the leader is within
    % close_enough of the current waypoint
    if norm(x(1:2, 1) - waypoint) < close_enough
        goal_distance = [goal_distance, [norm(x(1:2,1) - waypoint); toc(start_time)]]; %#ok<AGROW>
    end

    r.step();
end

%% =========================================================
%  SAVE DATA
%  =========================================================

save('DistanceData.mat', 'robot_distance');
save('GoalData.mat', 'goal_distance');

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();

%% =========================================================
%  HELPER FUNCTIONS
%  =========================================================

function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)
% DETERMINE_MARKER_SIZE  Returns a marker size in points corresponding to
% a desired physical size in meters.
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Points');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);

    marker_ratio = marker_size_meters / ...
                   (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));
    marker_size = cursize(3) * marker_ratio;
end

function font_size = determine_font_size(robotarium_instance, font_height_meters)
% DETERMINE_FONT_SIZE  Returns a font size in points corresponding to a
% desired physical height in meters.
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Pixels');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);

    font_ratio = font_height_meters / ...
                 (robotarium_instance.boundaries(4) - robotarium_instance.boundaries(3));
    font_size = cursize(4) * font_ratio;
end