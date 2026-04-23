% BARRIERS_WITH_PLOTTING
% Demonstrates single-integrator barrier certificates with a unicycle
% mapping. Ten robots swap positions on opposite sides of an ellipse three
% times while avoiding collisions via a unicycle barrier certificate.
%
% Visualization includes:
%   - Colored circle markers tracking each robot's position
%
% Good practice: when plotting time-varying animations for the Robotarium,
% update the data contained in existing plot objects every loop rather than
% replotting to reduce computation time. This is done in this script.
%
% Sean Wilson
% 3/2026

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 10;
safety_radius = 0.15;
projection_distance = 0.03;

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%% =========================================================
%  GOAL INITIALIZATION
%  =========================================================

% Distribute goal positions evenly around an ellipse. Robots alternate
% between the two halves of the ellipse on each swap.
xybound = [-1, 1, -0.8, 0.8];
p_theta = (1:2:2*N) / (2*N) * 2*pi;
p_circ = [xybound(2)*cos(p_theta),  xybound(2)*cos(p_theta + pi); ...
          xybound(4)*sin(p_theta),  xybound(4)*sin(p_theta + pi)];

x_goal = p_circ(:, 1:N);
flag = 0;   % 0 = first half of ellipse, 1 = second half
count = 0;   % number of completed swaps

%% =========================================================
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

% Single-integrator position controller drives agents to goal positions
position_int = create_si_position_controller( ...
    'XVelocityGain', 2, 'YVelocityGain', 2, 'VelocityMagnitudeLimit', 0.15);

% Paired SI<->unicycle mapping: projects SI control point ahead of unicycle
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping( ...
    'ProjectionDistance', projection_distance);

% Barrier certificate prevents inter-robot collisions
uni_barrier_certificate = create_uni_barrier_certificate( ...
    'BarrierGain', 500, 'SafetyRadius', safety_radius, ...
    'ProjectionDistance', projection_distance);

%% =========================================================
%  PLOT INITIALIZATION
%  =========================================================

% Get initial poses before any plotting so robot markers start in the
% right place. Must be called once before the first r.step().
x = r.get_poses();

% One color per robot, used consistently across all plot elements
CM = hsv2rgb([linspace(0, 1 - 1/N, N)', ones(N, 1)*0.8, ones(N, 1)*0.9]);

% Scale marker size to the figure window
marker_size_robot = determine_marker_size(r, 1.5 * safety_radius);

% Draw one circle marker per robot
g = gobjects(1, N);
for i = 1:N
    g(i) = plot(x(1,i), x(2,i), 'o', ...
        'MarkerSize', marker_size_robot, 'LineWidth', 7, 'Color', CM(i,:));
end

% Step once to render robots before the main loop begins
r.step();

%% =========================================================
%  MAIN SIMULATION LOOP
%  =========================================================

while true

    % Retrieve the most recent poses from the Robotarium.
    % The time delay is approximately 0.033 seconds.
    x  = r.get_poses();
    xi = uni_to_si_states(x);

    % ---------------------------------------------------------
    % Termination check: exit after three completed swaps
    % ---------------------------------------------------------
    if count == 3
        break;
    end

    % ---------------------------------------------------------
    % Goal selection: alternate between the two ellipse halves
    % ---------------------------------------------------------
    if flag == 0
        x_goal = p_circ(:, 1:N);
    else
        x_goal = p_circ(:, N+1:2*N);
    end

    % ---------------------------------------------------------
    % SI position controller drives agents toward current goals
    % ---------------------------------------------------------
    dxi = position_int(xi, x_goal);

    % ---------------------------------------------------------
    % Update robot marker positions
    % ---------------------------------------------------------
    for q = 1:N
        g(q).XData = x(1, q);
        g(q).YData = x(2, q);
    end

    % ---------------------------------------------------------
    % Convert SI velocities to unicycle commands and apply
    % barrier certificate to prevent collisions
    % ---------------------------------------------------------
    dxu = si_to_uni_dyn(dxi, x);

    % Stop any robot that has already reached its goal
    for j = 1:N
        if norm(x_goal(:,j) - xi(:,j), 1) < 0.05
            dxu(:, j) = [0; 0];
        end
    end

    dxu = uni_barrier_certificate(dxu, x);

    % ---------------------------------------------------------
    % Resize markers in case the figure window was resized.
    % This is a no-op when deployed on the physical Robotarium.
    % ---------------------------------------------------------
    [g.MarkerSize] = deal(determine_marker_size(r, 1.5 * safety_radius));

    % ---------------------------------------------------------
    % Send velocity commands and advance the simulation
    % ---------------------------------------------------------
    r.set_velocities(1:N, dxu);
    r.step();

    % ---------------------------------------------------------
    % Swap detection: if all robots are close enough, flip flag
    % ---------------------------------------------------------
    if norm(x_goal - xi, 1) < 0.07
        flag  = 1 - flag;
        count = count + 1;
    end
end

%% =========================================================
%  COMPLETION CAPTION
%  =========================================================

font_size = determine_font_size(r, 0.07);
finished_label = text(0, 0, sprintf('All robots safely reached \n their destination!'), ...
    'FontSize', font_size, 'Color', 'k', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
uistack(finished_label, 'top');

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