% DEAD_RECKONING_WITH_PLOTTING
% Drives N robots to randomized goal positions while estimating each
% robot's pose in real time using two dead reckoning methods:
%
%   1. Encoder-only  — unicycle model integrating wheel displacements
%   2. IMU + Encoder — encoder for translation, IMU yaw for heading
%
% Both estimated poses are displayed live alongside the Vicon ground truth
% using translucent ghost robots so drift is immediately visible.
%
% Sensor data is saved to .mat files at the end of the experiment for
% offline analysis with plot_comparison.m.
%
% Sean Wilson
% 03/2026

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 3;           % Number of robots
finalCount = 3;  % Number of goal sets to complete before stopping
goal_spacing = 0.5; % Minimum spacing between goal points (meters)

%% =========================================================
%  ROBOT PHYSICAL CONSTANTS
%  =========================================================

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%% =========================================================
%  ROBOT PHYSICAL CONSTANTS
%  =========================================================

% Pull directly from the Robotarium class so these stay in sync with
% the simulator if hardware parameters are ever updated
wheel_radius = r.wheel_radius;                                              % meters
base_length = r.base_length;                                                % meters
counts_per_wheel_rev = r.encoder_counts_per_revolution * r.motor_gear_ratio; % counts/wheel-rev
dt = r.time_step;                                                           % seconds

%% =========================================================
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

single_integrator_position_controller = create_si_position_controller();
si_barrier_cert = create_si_barrier_certificate_with_boundary();
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics();

% Goal-set checker: loose tolerance, only position matters
all_reached = create_at_position('PositionError', 0.10);

% Per-robot checker: tighter tolerance to zero individual velocities
at_goal_checker = create_at_position('PositionError', 0.08);

%% =========================================================
%  GOAL INITIALIZATION
%  =========================================================

arena_width = r.boundaries(2) - r.boundaries(1) - 3*r.robot_diameter;
arena_height = r.boundaries(4) - r.boundaries(3) - 3*r.robot_diameter;

goal_points = generate_random_positions(N, ...
    'Width', arena_width, ...
    'Height', arena_height, ...
    'Spacing', goal_spacing);

%% =========================================================
%  DEAD RECKONING STATE INITIALIZATION
%  =========================================================

% Both estimators are initialized from the true pose at t=0.
% encoder_pose(:, i)     — current estimated pose from encoder-only model
% imu_encoder_pose(:, i) — current estimated pose from IMU+encoder model
% encoder_ref(:, i)      — encoder tick counts at initialization (to zero out)
% imu_yaw_offset(i)      — IMU yaw at t=0, used to align IMU to Vicon heading

x_init = r.get_poses();
r.step();

encoder_pose = x_init;
imu_encoder_pose = x_init;
encoder_ref = double(r.get_encoders());
imu_yaw_offset = zeros(1, N);

% Align IMU yaw to Vicon heading at t=0
raw_imu = r.get_orientations(); % degrees
for i = 1:N
    imu_yaw_offset(i) = deg2rad(raw_imu(1, i)) - x_init(3, i);
end

%% =========================================================
%  PLOT INITIALIZATION
%  =========================================================

% One color per robot, consistent across all plot elements
CM = lines(N);

% Scale marker size to figure window
font_size = determine_font_size(r, 0.04);
line_width = 3;

% Robot diameter in axes units for drawing ghost circles
robot_radius = r.robot_diameter / 2;
theta_circle = linspace(0, 2*pi, 32);
circle_x = robot_radius * cos(theta_circle);
circle_y = robot_radius * sin(theta_circle);

% Pre-allocate plot handles for real-time updates
% Each robot gets:
%   truth_dot       — solid filled circle at true position
%   enc_ghost       — translucent circle at encoder-only estimate
%   imu_ghost       — translucent circle at IMU+encoder estimate
%   enc_arrow       — heading arrow for encoder estimate
%   imu_arrow       — heading arrow for IMU+encoder estimate
%   truth_trail     — faint line showing true path
%   enc_trail       — faint line showing encoder-only path
%   imu_trail       — faint line showing IMU+encoder path

for i = 1:N
    c = CM(i, :);

    % Ground truth: solid circle
    truth_dot(i) = plot(x_init(1,i), x_init(2,i), 'o', ...
        'MarkerSize', 14, 'MarkerFaceColor', c, 'MarkerEdgeColor', c * 0.6, ...
        'LineWidth', 3);

    % Encoder-only ghost: hollow circle, same color, translucent via alpha patch
    enc_ghost(i) = patch(x_init(1,i) + circle_x, x_init(2,i) + circle_y, c, ...
        'FaceAlpha', 0.25, 'EdgeColor', c, 'EdgeAlpha', 0.6, 'LineWidth', 3, 'LineStyle', '--');

    % IMU+encoder ghost: hollow circle, desaturated color
    imu_color = 0.5*c + 0.5*[1 1 1]; % lighten
    imu_ghost(i) = patch(x_init(1,i) + circle_x, x_init(2,i) + circle_y, imu_color, ...
        'FaceAlpha', 0.25, 'EdgeColor', imu_color*0.7, 'EdgeAlpha', 0.6, 'LineWidth', 3, 'LineStyle', ':');

    % Heading arrows (quiver): encoder=dashed, IMU=dotted
    arrow_len = 0.08;
    enc_arrow(i) = quiver(x_init(1,i), x_init(2,i), ...
        arrow_len*cos(x_init(3,i)), arrow_len*sin(x_init(3,i)), ...
        'Color', c, 'LineWidth', 3, 'MaxHeadSize', 3, 'AutoScale', 'off');
    imu_arrow(i) = quiver(x_init(1,i), x_init(2,i), ...
        arrow_len*cos(x_init(3,i)), arrow_len*sin(x_init(3,i)), ...
        'Color', imu_color*0.7, 'LineWidth', 3, 'MaxHeadSize', 3, 'AutoScale', 'off');

    % Trajectory trails
    truth_trail(i) = plot(x_init(1,i), x_init(2,i), '-', 'Color', [c, 0.4], 'LineWidth', 2);
    enc_trail(i)   = plot(x_init(1,i), x_init(2,i), '--', 'Color', [c, 0.25], 'LineWidth', 2);
    imu_trail(i)   = plot(x_init(1,i), x_init(2,i), ':', 'Color', [imu_color, 0.25], 'LineWidth', 2);

    % Robot label
    robot_labels{i} = text(x_init(1,i)+0.05, x_init(2,i)+0.05, sprintf('R%d', i), ...
        'FontSize', font_size, 'FontWeight', 'bold', 'Color', c*0.7);
end

% Legend entries (invisible markers just for the legend)
hold on;
h_truth = plot(NaN, NaN, 'o', 'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerEdgeColor', 'k', 'DisplayName', 'Vicon (truth)');
h_enc   = plot(NaN, NaN, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 3, 'DisplayName', 'Encoder only');
h_imu   = plot(NaN, NaN, ':',  'Color', [0.6 0.6 0.6], 'LineWidth', 3, 'DisplayName', 'IMU + Encoder');
% Manually position the legend centered near the top of the arena.
% Setting Units to 'normalized' then centering horizontally and placing
% near the top ensures it sits inside the +y boundary regardless of
% figure window size.
lg = legend([h_truth, h_enc, h_imu], 'Orientation', 'horizontal', ...
    'FontSize', font_size, 'Box', 'on');
lg.Units = 'normalized';
lg.Position(1) = 0.5 - lg.Position(3)/2; % Center horizontally
lg.Position(2) = 0.85;                    % Near top of arena (+y boundary)

%% =========================================================
%  DATA STORAGE INITIALIZATION
%  =========================================================

% Pre-populate timestep 0 from x_init and the sensor readings taken
% immediately after the first r.step(). This ensures plot_comparison
% can use vicon_pose_array(:, robot, 1) as the true initial pose
% without needing a separate init_data.mat file.
vicon_pose_array      = x_init;
imu_orientation_array = r.get_orientations();
encoder_array         = double(r.get_encoders());
gyro_array            = r.get_gyros();
acc_array             = r.get_accelerations();
mag_array             = r.get_magnetic_fields();

%% =========================================================
%  MAIN EXPERIMENT LOOP
%  =========================================================

count = 0;

while true

    x = r.get_poses();
    x_si = uni_to_si_states(x);

    % ---------------------------------------------------------
    % Record sensor data
    % ---------------------------------------------------------
    vicon_pose_array(:, :, end+1) = x;                         %#ok<SAGROW>
    imu_orientation_array(:, :, end+1) = r.get_orientations(); %#ok<SAGROW>
    encoder_array(:, :, end+1) = r.get_encoders();             %#ok<SAGROW>
    gyro_array(:, :, end+1) = r.get_gyros();                   %#ok<SAGROW>
    acc_array(:, :, end+1) = r.get_accelerations();            %#ok<SAGROW>
    mag_array(:, :, end+1) = r.get_magnetic_fields();          %#ok<SAGROW>

    % ---------------------------------------------------------
    % Dead reckoning update for each robot
    % ---------------------------------------------------------
    raw_enc = double(r.get_encoders()) - encoder_ref;
    raw_imu = r.get_orientations(); % degrees, row 1 = yaw

    for i = 1:N
        % --- Encoder tick deltas since last step ---
        T = size(encoder_array, 3);
        if T >= 2
            d_counts = double(encoder_array(:, i, end)) - double(encoder_array(:, i, end-1));
        else
            d_counts = [0; 0];
        end

        % Convert tick deltas to wheel displacements (meters)
        d_left  = (d_counts(1) / counts_per_wheel_rev) * 2 * pi * wheel_radius;
        d_right = (d_counts(2) / counts_per_wheel_rev) * 2 * pi * wheel_radius;
        d_center = (d_right + d_left) / 2;

        % --- Encoder-only unicycle model ---
        d_theta = (d_right - d_left) / base_length;
        theta_enc = encoder_pose(3, i);
        encoder_pose(1, i) = encoder_pose(1, i) + d_center * cos(theta_enc + d_theta/2);
        encoder_pose(2, i) = encoder_pose(2, i) + d_center * sin(theta_enc + d_theta/2);
        encoder_pose(3, i) = theta_enc + d_theta;

        % --- IMU + encoder model: encoder for translation, IMU for heading ---
        imu_yaw_rad = deg2rad(raw_imu(1, i)) - imu_yaw_offset(i);
        imu_encoder_pose(1, i) = imu_encoder_pose(1, i) + d_center * cos(imu_yaw_rad);
        imu_encoder_pose(2, i) = imu_encoder_pose(2, i) + d_center * sin(imu_yaw_rad);
        imu_encoder_pose(3, i) = imu_yaw_rad;
    end

    % ---------------------------------------------------------
    % Update plot handles
    % ---------------------------------------------------------
    arrow_len = 0.08;
    for i = 1:N
        % Ground truth
        truth_dot(i).XData = x(1, i);
        truth_dot(i).YData = x(2, i);
        truth_trail(i).XData = [truth_trail(i).XData, x(1, i)];
        truth_trail(i).YData = [truth_trail(i).YData, x(2, i)];

        % Encoder-only ghost circle and arrow
        enc_ghost(i).XData = encoder_pose(1, i) + circle_x;
        enc_ghost(i).YData = encoder_pose(2, i) + circle_y;
        enc_arrow(i).XData = encoder_pose(1, i);
        enc_arrow(i).YData = encoder_pose(2, i);
        enc_arrow(i).UData = arrow_len * cos(encoder_pose(3, i));
        enc_arrow(i).VData = arrow_len * sin(encoder_pose(3, i));
        enc_trail(i).XData = [enc_trail(i).XData, encoder_pose(1, i)];
        enc_trail(i).YData = [enc_trail(i).YData, encoder_pose(2, i)];

        % IMU+encoder ghost circle and arrow
        imu_ghost(i).XData = imu_encoder_pose(1, i) + circle_x;
        imu_ghost(i).YData = imu_encoder_pose(2, i) + circle_y;
        imu_arrow(i).XData = imu_encoder_pose(1, i);
        imu_arrow(i).YData = imu_encoder_pose(2, i);
        imu_arrow(i).UData = arrow_len * cos(imu_encoder_pose(3, i));
        imu_arrow(i).VData = arrow_len * sin(imu_encoder_pose(3, i));
        imu_trail(i).XData = [imu_trail(i).XData, imu_encoder_pose(1, i)];
        imu_trail(i).YData = [imu_trail(i).YData, imu_encoder_pose(2, i)];

        % Robot label follows truth
        robot_labels{i}.Position = x(1:2, i) + [0.05; 0.05];
    end

    % ---------------------------------------------------------
    % Goal-set completion check
    % ---------------------------------------------------------
    if all_reached(x_si, goal_points)
        count = count + 1;
        if count == finalCount
            break;
        end
        goal_points = generate_random_positions(N, ...
            'Width', arena_width, ...
            'Height', arena_height, ...
            'Spacing', goal_spacing);
    end

    % ---------------------------------------------------------
    % Compute and apply control inputs
    % ---------------------------------------------------------
    dxi = single_integrator_position_controller(x_si, goal_points);

    for k = 1:N
        if at_goal_checker(x(:, k), goal_points(:, k))
            dxi(:, k) = zeros(2, 1);
        end
    end

    dxi = si_barrier_cert(dxi, x_si);
    dxu = si_to_uni_dyn(dxi, x);
    r.set_velocities(1:N, dxu);
    r.step();
end

%% =========================================================
%  SAVE DATA
%  =========================================================

save('vicon_pose_array.mat',      'vicon_pose_array');
save('imu_orientation_array.mat', 'imu_orientation_array');
save('encoder_array.mat',         'encoder_array');
save('gyro_array.mat',            'gyro_array');
save('acc_array.mat',             'acc_array');
save('mag_array.mat',             'mag_array');

%% =========================================================
%  DEBUG REPORT
%  =========================================================

r.debug();

%% =========================================================
%  HELPER FUNCTIONS
%  =========================================================

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