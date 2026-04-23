% GO_TO_POSE_WITH_PLOTTING
% This is the same code as go_to_pose.m with some plotting routines
% included. Drives N robots to randomized goal poses within the Robotarium
% arena. When all robots reach their goals, new goals are chosen and the
% process repeats for the duration of the simulation.
%
% Visualization includes:
%   - Colored circle markers tracking each robot's position
%   - Colored square markers and orientation arrows at each goal
%   - Per-robot position readouts
%   - Iteration count and elapsed time display
%   - GT Logo as background image
%
% Good practice: when plotting time varying animations for the Robotarium,
% update the data contained in existing plot objects every loop rather than
% replotting to reduce computation time. This is done in this script.
%
% Sean Wilson
% 07/2019

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 6;           % Number of robots
iterations = 5000; % Total simulation steps

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

% Controller drives each robot to a target pose [x; y; theta]
args = {'PositionError', 0.05, 'RotationError', 0.2};
controller = create_pose_parking_controller_clf(args{:});
init_checker = create_at_pose(args{:});

%% =========================================================
%  GOAL INITIALIZATION
%  =========================================================

% Keep goals away from the arena edge by one robot diameter on each side
arena_width = r.boundaries(2) - r.boundaries(1) - 3*r.robot_diameter;
arena_height = r.boundaries(4) - r.boundaries(3) - 3*r.robot_diameter;

final_goal_points = generate_random_poses(N, ...
    'Width',   arena_width, ...
    'Height',  arena_height, ...
    'Spacing', 0.5);

%% =========================================================
%  PLOT INITIALIZATION
%  =========================================================

% Get initial poses before any plotting so robot markers start in the
% right place. Must be called once before the first r.step().
x = r.get_poses();

% One color per robot, used consistently across all plot elements
CM = rand(N, 3);

% Scale marker and font sizes to the figure window so they look correct
% both in simulation and when deployed on the physical Robotarium
marker_size_goal = determine_marker_size(r, 0.20);
marker_size_robot = determine_robot_marker_size(r);
font_size = determine_font_size(r, 0.05);
line_width = 5;

% Pre-allocate plot handle arrays
d = gobjects(1, N); % Goal square markers
a = gobjects(1, N); % Goal orientation arrows
g = gobjects(1, N); % Robot position circles
goal_labels = cell(1, N);
robot_labels = cell(1, N);
robot_details_text = cell(1, N);

% Draw per-robot plot elements
for i = 1:N
    % --- Goal markers ---
    d(i) = plot(final_goal_points(1,i), final_goal_points(2,i), ...
        's', 'MarkerSize', marker_size_goal, 'LineWidth', line_width, 'Color', CM(i,:));

    a(i) = quiver(final_goal_points(1,i), final_goal_points(2,i), ...
        0.12*cos(final_goal_points(3,i)), 0.12*sin(final_goal_points(3,i)), ...
        'LineWidth', line_width, 'MaxHeadSize', 2*line_width, 'Color', CM(i,:));

    goal_labels{i} = text(final_goal_points(1,i)-0.05, final_goal_points(2,i), ...
        sprintf('G%d', i), 'FontSize', font_size, 'FontWeight', 'bold');

    % --- Robot markers ---
    g(i) = plot(x(1,i), x(2,i), ...
        'o', 'MarkerSize', marker_size_robot, 'LineWidth', 5, 'Color', CM(i,:));

    % Labels are placed off-screen initially and repositioned each step
    robot_labels{i} = text(500, 500, ...
        sprintf('Robot %d', i), 'FontSize', font_size, 'FontWeight', 'bold');

    robot_details_text{i} = text(500, 500, ...
        sprintf('X-Pos: %d \nY-Pos: %d', x(1,i), x(2,i)), ...
        'FontSize', font_size, 'FontWeight', 'bold');
end

% Draw iteration counter and elapsed time in the lower-left corner.
% Note: on your local machine this reflects wall-clock time. Multiply
% iteration count by 0.033 for a better estimate of physical experiment time.
start_time = tic;
iteration_label = text(-1.5, -0.8, sprintf('Iteration %d', 0), ...
    'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');
time_label = text(-1.5, -0.9, sprintf('Total Time Elapsed %0.2f', toc(start_time)), ...
    'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');

% Draw GT logo as background image.
% y_img runs 1 -> -1 because the image (1,1) pixel is in the top-left corner.
gt_img = imread('GTLogo.png');
x_img = linspace(-1.0, 1.0, size(gt_img, 2));
y_img = linspace(1.0, -1.0, size(gt_img, 1));
gt_img_handle = image(x_img, y_img, gt_img, 'CDataMapping', 'scaled');

% Step once to render robots before setting layer order, so all objects
% exist in the axes when uistack is called.
r.step();

% Set draw order: image at the bottom, labels and captions on top
uistack(a, 'bottom');               % Arrows just above image
uistack([goal_labels{:}], 'bottom'); % Goal labels above arrows
uistack(d, 'bottom');               % Goal squares at the bottom
uistack(iteration_label, 'top');    % Iteration counter on top
uistack(time_label, 'top');         % Time label above iteration counter
uistack(gt_img_handle, 'bottom');   % Image below everything

%% =========================================================
%  MAIN SIMULATION LOOP
%  =========================================================

for i = 1:iterations

    x = r.get_poses();

    if ~init_checker(x, final_goal_points)
        % ---------------------------------------------------------
        % Robots still moving: update robot marker positions and
        % position readout text
        % ---------------------------------------------------------
        for q = 1:N
            g(q).XData = x(1, q);
            g(q).YData = x(2, q);

            robot_labels{q}.Position = x(1:2, q) + [-0.15; 0.15];
            robot_details_text{q}.String = sprintf('X-Pos: %0.2f \nY-Pos: %0.2f', x(1,q), x(2,q));
            robot_details_text{q}.Position = x(1:2, q) - [0.20; 0.25];
        end

    else
        % ---------------------------------------------------------
        % All robots reached their goals: generate new goal poses
        % and update goal markers
        % ---------------------------------------------------------
        final_goal_points = generate_random_poses(N, ...
            'Width',   arena_width, ...
            'Height',  arena_height, ...
            'Spacing', 0.5);

        for j = 1:N
            goal_labels{j}.Position = final_goal_points(1:2, j) - [0.05; 0];

            d(j).XData = final_goal_points(1, j);
            d(j).YData = final_goal_points(2, j);

            a(j).XData = final_goal_points(1, j);
            a(j).YData = final_goal_points(2, j);
            a(j).UData = 0.12 * cos(final_goal_points(3, j));
            a(j).VData = 0.12 * sin(final_goal_points(3, j));
        end
    end

    % ---------------------------------------------------------
    % Update iteration counter and elapsed time display
    % ---------------------------------------------------------
    iteration_label.String = sprintf('Iteration %d', i);
    time_label.String = sprintf('Total Time Elapsed %0.2f', toc(start_time));

    % ---------------------------------------------------------
    % Resize markers and fonts in case the figure window was
    % resized. This is a no-op when deployed on the Robotarium
    % since the figure window is fixed size.
    % ---------------------------------------------------------
    new_goal_size = determine_marker_size(r, 0.20);
    new_robot_size = determine_robot_marker_size(r);
    font_size = determine_font_size(r, 0.05);

    [d.MarkerSize] = deal(new_goal_size);
    [g.MarkerSize] = deal(new_robot_size);

    iteration_label.FontSize = font_size;
    time_label.FontSize = font_size;
    for k = 1:N
        robot_labels{k}.FontSize = font_size;
        goal_labels{k}.FontSize = font_size;
        robot_details_text{k}.FontSize = font_size;
    end

    % ---------------------------------------------------------
    % Compute and apply control inputs
    % ---------------------------------------------------------
    dxu = controller(x, final_goal_points);
    dxu = unicycle_barrier_certificate(dxu, x);
    r.set_velocities(1:N, dxu);
    r.step();
end

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();

%% =========================================================
%  HELPER FUNCTIONS
%  =========================================================

function marker_size = determine_robot_marker_size(robotarium_instance)
% DETERMINE_ROBOT_MARKER_SIZE  Returns a marker size in points that
% corresponds to the physical robot diameter in the figure window.
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Pixels');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);

    robot_ratio = (robotarium_instance.robot_diameter + 0.03) / ...
                   (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));
    marker_size = cursize(3) * robot_ratio;
end

function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)
% DETERMINE_MARKER_SIZE  Returns a marker size in points corresponding to
% a desired physical size in meters.
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Pixels');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);

    marker_ratio = marker_size_meters / ...
                   (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));
    marker_size  = cursize(3) * marker_ratio;
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
    font_size  = cursize(4) * font_ratio;
end
