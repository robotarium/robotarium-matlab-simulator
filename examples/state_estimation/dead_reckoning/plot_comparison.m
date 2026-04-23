% PLOT_COMPARISON
% Offline analysis script for dead reckoning data collected by
% dead_reckoning_with_plotting.m. Loads saved sensor arrays and compares
% two pose estimation methods against Vicon ground truth for each robot:
%
%   1. Encoder-only  — unicycle model integrating wheel displacements
%   2. IMU + Encoder — encoder for translation, IMU yaw for heading
%
% Produces two figures per robot:
%   Figure 1 — Yaw comparison: Vicon vs IMU vs integrated gyro, plus
%              per-sensor yaw error plots
%   Figure 2 — Global position comparison: X, Y, and 2D trajectory
%              for Vicon, encoder-only, and IMU+encoder estimates
%
% Summary error statistics are printed to the console for each robot.

%% =========================================================
%  ROBOT PHYSICAL CONSTANTS
%  =========================================================

% These must match the values used during data collection in
% dead_reckoning_with_plotting.m. If you have a Robotarium instance
% available, load them from r.wheel_radius etc. instead.
wheel_radius         = 0.016;        % meters
base_length          = 0.105;        % meters
counts_per_wheel_rev = 28 * 100.37;  % counts/motor-rev * gear ratio
dt                   = 0.033;        % Robotarium timestep (seconds)

%% =========================================================
%  DATA LOADING
%  =========================================================

vicon_pose_array      = load('vicon_pose_array.mat').vicon_pose_array;
imu_orientation_array = load('imu_orientation_array.mat').imu_orientation_array;
encoder_array         = load('encoder_array.mat').encoder_array;
gyro_array            = load('gyro_array.mat').gyro_array;

% Expected array sizes: 3 x N x T (poses/IMU/gyro) or 2 x N x T (encoders)
fprintf('Vicon shape:   %s\n', mat2str(size(vicon_pose_array)));
fprintf('IMU shape:     %s\n', mat2str(size(imu_orientation_array)));
fprintf('Gyro shape:    %s\n', mat2str(size(gyro_array)));
fprintf('Encoder shape: %s\n', mat2str(size(encoder_array)));

N = size(vicon_pose_array, 2); % Number of robots
T = size(vicon_pose_array, 3); % Number of timesteps
time = (0:T-1) * dt;           % Time axis in seconds

%% =========================================================
%  PER-ROBOT ANALYSIS
%  =========================================================

for robot = 1:N

    % ---------------------------------------------------------
    % Extract data for this robot
    % ---------------------------------------------------------
    vicon_yaw_rad = squeeze(vicon_pose_array(3, robot, :))';  % 1 x T, radians in [-pi, pi]
    imu_yaw_deg   = squeeze(imu_orientation_array(1, robot, :))'; % 1 x T, degrees in [0, 360]
    gyro_z        = squeeze(gyro_array(3, robot, :))';        % 1 x T, rad/s
    enc           = reshape(encoder_array(:, robot, :), 2, []);  % 2 x T, safe for any N

    % ---------------------------------------------------------
    % Yaw signal processing
    % ---------------------------------------------------------

    % Unwrap Vicon yaw (radians) then convert to degrees.
    % Vicon outputs in [-pi, pi] so unwrap before converting to avoid
    % a false discontinuity at the +/-180 degree boundary.
    vicon_yaw_deg = rad2deg(unwrap(vicon_yaw_rad));

    % Integrate gyro rate (rad/s) -> cumulative yaw in degrees
    gyro_yaw_deg = cumsum(gyro_z * dt) * (180 / pi);

    % The IMU outputs orientation in [0, 360]. Convert to [-180, 180]
    % to match the Vicon convention before unwrapping, otherwise unwrap
    % sees a false 360-degree discontinuity at startup.
    imu_yaw_deg = mod(imu_yaw_deg + 180, 360) - 180;

    % Unwrap IMU and gyro in degree space (period = 360)
    imu_yaw_deg  = unwrap(imu_yaw_deg,  360);
    gyro_yaw_deg = unwrap(gyro_yaw_deg, 360);

    % Align IMU and gyro to Vicon at t=0 so all three signals share the
    % same reference heading
    imu_yaw_deg  = (imu_yaw_deg  - imu_yaw_deg(1))  + vicon_yaw_deg(1);
    gyro_yaw_deg = (gyro_yaw_deg - gyro_yaw_deg(1)) + vicon_yaw_deg(1);

    % ---------------------------------------------------------
    % Dead reckoning pose estimation
    % ---------------------------------------------------------
    vicon_init = squeeze(vicon_pose_array(:, robot, 1)); % [x; y; theta] at t=0

    global_pose     = estimate_pose_encoder(enc, vicon_init, ...
                        wheel_radius, base_length, counts_per_wheel_rev);

    imu_yaw_rad_aligned = deg2rad(imu_yaw_deg);
    global_pose_imu = estimate_pose_imu_encoder(enc, imu_yaw_rad_aligned, vicon_init, ...
                        wheel_radius, counts_per_wheel_rev);

    % ---------------------------------------------------------
    % Trim all signals to the same length
    % ---------------------------------------------------------
    n = min([T, length(imu_yaw_deg), length(gyro_yaw_deg)]);
    m = min([T, size(global_pose, 2), size(global_pose_imu, 2)]);
    t_n = time(1:n);
    t_m = time(1:m);

    % ---------------------------------------------------------
    % Figure 1: Yaw comparison
    % ---------------------------------------------------------
    fig1 = figure('Name', sprintf('Robot %d — Yaw', robot), ...
        'NumberTitle', 'off', 'Position', [100, 100, 900, 800]);

    % Subplot 1: raw yaw signals
    ax1 = subplot(3, 1, 1);
    plot(ax1, t_n, vicon_yaw_deg(1:n),  'b-',  'LineWidth', 1.5, 'DisplayName', 'Vicon (Truth)');
    hold(ax1, 'on');
    plot(ax1, t_n, imu_yaw_deg(1:n),   '--',  'Color', [0.85 0.55 0.0], 'LineWidth', 1.5, 'DisplayName', 'IMU Estimate');
    plot(ax1, t_n, gyro_yaw_deg(1:n),  ':',   'Color', [0.1 0.6 0.1],   'LineWidth', 1.5, 'DisplayName', 'Gyro Integrated');
    title(ax1, sprintf('Robot %d — Yaw: Vicon vs IMU vs Integrated Gyro', robot));
    xlabel(ax1, 'Time (s)');
    ylabel(ax1, 'Yaw (degrees)');
    legend(ax1, 'Location', 'best');
    grid(ax1, 'on');

    % Subplot 2: IMU yaw error
    imu_error = imu_yaw_deg(1:n) - vicon_yaw_deg(1:n);
    ax2 = subplot(3, 1, 2);
    plot(ax2, t_n, imu_error, '--', 'Color', [0.85 0.55 0.0], 'LineWidth', 1.5, 'DisplayName', 'IMU Error (IMU - Vicon)');
    hold(ax2, 'on');
    yline(ax2, 0, 'k--', 'LineWidth', 0.8);
    title(ax2, 'IMU Yaw Error');
    xlabel(ax2, 'Time (s)');
    ylabel(ax2, 'Error (degrees)');
    legend(ax2, 'Location', 'best');
    grid(ax2, 'on');

    % Subplot 3: gyro yaw error
    gyro_error = gyro_yaw_deg(1:n) - vicon_yaw_deg(1:n);
    ax3 = subplot(3, 1, 3);
    plot(ax3, t_n, gyro_error, ':', 'Color', [0.1 0.6 0.1], 'LineWidth', 1.5, 'DisplayName', 'Gyro Error (Gyro - Vicon)');
    hold(ax3, 'on');
    yline(ax3, 0, 'k--', 'LineWidth', 0.8);
    title(ax3, 'Integrated Gyro Yaw Error');
    xlabel(ax3, 'Time (s)');
    ylabel(ax3, 'Error (degrees)');
    legend(ax3, 'Location', 'best');
    grid(ax3, 'on');

    sgtitle(fig1, sprintf('Robot %d — Yaw Comparison', robot), ...
        'FontSize', 14, 'FontWeight', 'bold');
    saveas(fig1, sprintf('yaw_comparison_robot_%d.png', robot));

    % ---------------------------------------------------------
    % Figure 2: Global position comparison
    % ---------------------------------------------------------
    fig2 = figure('Name', sprintf('Robot %d — Position', robot), ...
        'NumberTitle', 'off', 'Position', [200, 100, 900, 900]);

    % Subplot 1: X position
    ax4 = subplot(3, 1, 1);
    plot(ax4, t_m, squeeze(vicon_pose_array(1, robot, 1:m))', 'b-',  'LineWidth', 1.5, 'DisplayName', 'Vicon (Truth)');
    hold(ax4, 'on');
    plot(ax4, t_m, global_pose(1, 1:m),     'r--', 'LineWidth', 1.5, 'DisplayName', 'Encoder Only');
    plot(ax4, t_m, global_pose_imu(1, 1:m), ':',   'Color', [0.5 0.0 0.8], 'LineWidth', 1.5, 'DisplayName', 'IMU+Enc Odom');
    title(ax4, 'X Position');
    xlabel(ax4, 'Time (s)');
    ylabel(ax4, 'X (m)');
    legend(ax4, 'Location', 'best');
    grid(ax4, 'on');

    % Subplot 2: Y position
    ax5 = subplot(3, 1, 2);
    plot(ax5, t_m, squeeze(vicon_pose_array(2, robot, 1:m))', 'b-',  'LineWidth', 1.5, 'DisplayName', 'Vicon (Truth)');
    hold(ax5, 'on');
    plot(ax5, t_m, global_pose(2, 1:m),     'r--', 'LineWidth', 1.5, 'DisplayName', 'Encoder Only');
    plot(ax5, t_m, global_pose_imu(2, 1:m), ':',   'Color', [0.5 0.0 0.8], 'LineWidth', 1.5, 'DisplayName', 'IMU+Enc Odom');
    title(ax5, 'Y Position');
    xlabel(ax5, 'Time (s)');
    ylabel(ax5, 'Y (m)');
    legend(ax5, 'Location', 'best');
    grid(ax5, 'on');

    % Subplot 3: 2D trajectory
    ax6 = subplot(3, 1, 3);
    vicon_x = squeeze(vicon_pose_array(1, robot, 1:m))';
    vicon_y = squeeze(vicon_pose_array(2, robot, 1:m))';
    plot(ax6, vicon_x,              vicon_y,              'b-',  'LineWidth', 1.5, 'DisplayName', 'Vicon (Truth)');
    hold(ax6, 'on');
    plot(ax6, global_pose(1, 1:m),     global_pose(2, 1:m),     'r--', 'LineWidth', 1.5, 'DisplayName', 'Encoder Only');
    plot(ax6, global_pose_imu(1, 1:m), global_pose_imu(2, 1:m), ':',   'Color', [0.5 0.0 0.8], 'LineWidth', 1.5, 'DisplayName', 'IMU+Enc Odom');
    scatter(ax6, vicon_x(1),   vicon_y(1),   80, 'b', 'o', 'filled', 'DisplayName', 'Start');
    scatter(ax6, vicon_x(end), vicon_y(end), 80, 'b', 'x', 'LineWidth', 2, 'DisplayName', 'End');
    title(ax6, '2D Trajectory');
    xlabel(ax6, 'X (m)');
    ylabel(ax6, 'Y (m)');
    legend(ax6, 'Location', 'best');
    grid(ax6, 'on');
    xlim(ax6, [-1.6, 1.6]);
    ylim(ax6, [-1.0, 1.0]);
    set(ax6, 'PlotBoxAspectRatio', [3.2 2.0 1]); % Match 3.2m x 2.0m arena proportions

    sgtitle(fig2, sprintf('Robot %d — Global Position Comparison', robot), ...
        'FontSize', 14, 'FontWeight', 'bold');
    saveas(fig2, sprintf('position_comparison_robot_%d.png', robot));

    % ---------------------------------------------------------
    % Summary statistics
    % ---------------------------------------------------------
    fprintf('\nYaw Error Stats for Robot %d (degrees):\n', robot);
    fprintf('  IMU  - Mean: %.4f  Std: %.4f  Max: %.4f\n', ...
        mean(imu_error), std(imu_error), max(abs(imu_error)));
    fprintf('  Gyro - Mean: %.4f  Std: %.4f  Max: %.4f\n', ...
        mean(gyro_error), std(gyro_error), max(abs(gyro_error)));

    x_error = global_pose(1, 1:m) - squeeze(vicon_pose_array(1, robot, 1:m))';
    y_error = global_pose(2, 1:m) - squeeze(vicon_pose_array(2, robot, 1:m))';
    x_error_imu = global_pose_imu(1, 1:m) - squeeze(vicon_pose_array(1, robot, 1:m))';
    y_error_imu = global_pose_imu(2, 1:m) - squeeze(vicon_pose_array(2, robot, 1:m))';

    fprintf('\nPosition Error Stats for Robot %d (meters):\n', robot);
    fprintf('  Encoder Only - X: Mean %.4f  Std %.4f  Max %.4f\n', mean(x_error), std(x_error), max(abs(x_error)));
    fprintf('  Encoder Only - Y: Mean %.4f  Std %.4f  Max %.4f\n', mean(y_error), std(y_error), max(abs(y_error)));
    fprintf('  IMU+Encoder  - X: Mean %.4f  Std %.4f  Max %.4f\n', mean(x_error_imu), std(x_error_imu), max(abs(x_error_imu)));
    fprintf('  IMU+Encoder  - Y: Mean %.4f  Std %.4f  Max %.4f\n', mean(y_error_imu), std(y_error_imu), max(abs(y_error_imu)));
end

%% =========================================================
%  HELPER FUNCTIONS
%  =========================================================

function global_pose = estimate_pose_encoder(encoder, vicon_init, wheel_radius, base_length, counts_per_wheel_rev)
% ESTIMATE_POSE_ENCODER  Estimates global pose using a unicycle model of a
% differential drive robot. Encoder readings are zeroed at t=0 relative to
% the initial Vicon pose. Both translation and heading come from encoders.
%
%   encoder            — 2 x T array of raw encoder counts [left; right]
%   vicon_init         — 3 x 1 initial pose [x; y; theta] from Vicon (rad)
%   wheel_radius       — wheel radius in meters
%   base_length        — wheel-to-wheel distance in meters
%   counts_per_wheel_rev — encoder counts per full wheel revolution
%
%   Returns global_pose — 3 x T array of estimated [x; y; theta]

    T = size(encoder, 2);
    global_pose = zeros(3, T);
    global_pose(:, 1) = vicon_init;

    % Zero encoder counts at t=0
    encoder_zeroed = encoder - encoder(:, 1);

    for t = 2:T
        d_counts_left  = encoder_zeroed(1, t) - encoder_zeroed(1, t-1);
        d_counts_right = encoder_zeroed(2, t) - encoder_zeroed(2, t-1);

        d_left  = (d_counts_left  / counts_per_wheel_rev) * 2 * pi * wheel_radius;
        d_right = (d_counts_right / counts_per_wheel_rev) * 2 * pi * wheel_radius;

        d_center = (d_right + d_left) / 2;
        d_theta  = (d_right - d_left) / base_length;

        theta = global_pose(3, t-1);
        global_pose(1, t) = global_pose(1, t-1) + d_center * cos(theta + d_theta/2);
        global_pose(2, t) = global_pose(2, t-1) + d_center * sin(theta + d_theta/2);
        global_pose(3, t) = theta + d_theta;
    end
end

function global_pose = estimate_pose_imu_encoder(encoder, imu_yaw_rad, vicon_init, wheel_radius, counts_per_wheel_rev)
% ESTIMATE_POSE_IMU_ENCODER  Estimates global pose using encoder wheel
% displacements for translation and IMU absolute orientation for heading.
% This avoids heading drift from encoder slippage at the cost of IMU bias.
%
%   encoder            — 2 x T array of raw encoder counts [left; right]
%   imu_yaw_rad        — 1 x T array of IMU yaw in radians, aligned to Vicon at t=0
%   vicon_init         — 3 x 1 initial pose [x; y; theta] from Vicon (rad)
%   wheel_radius       — wheel radius in meters
%   counts_per_wheel_rev — encoder counts per full wheel revolution
%
%   Returns global_pose — 3 x T array of estimated [x; y; theta]

    T = size(encoder, 2);
    global_pose = zeros(3, T);
    global_pose(:, 1) = vicon_init;

    % Zero encoder counts at t=0
    encoder_zeroed = encoder - encoder(:, 1);

    for t = 2:T
        d_counts_left  = encoder_zeroed(1, t) - encoder_zeroed(1, t-1);
        d_counts_right = encoder_zeroed(2, t) - encoder_zeroed(2, t-1);

        d_left  = (d_counts_left  / counts_per_wheel_rev) * 2 * pi * wheel_radius;
        d_right = (d_counts_right / counts_per_wheel_rev) * 2 * pi * wheel_radius;

        d_center = (d_right + d_left) / 2;

        % Use IMU for heading rather than encoder-derived d_theta
        theta = imu_yaw_rad(t);
        global_pose(1, t) = global_pose(1, t-1) + d_center * cos(theta);
        global_pose(2, t) = global_pose(2, t-1) + d_center * sin(theta);
        global_pose(3, t) = theta;
    end
end