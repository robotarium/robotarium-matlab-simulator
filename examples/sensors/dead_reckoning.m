% DEAD_RECKONING
% Drives N robots to randomized goal positions, recording IMU, encoder,
% gyro, magnetometer, and pose data every iteration. Robots cycle through
% finalCount sets of randomized goal points. All sensor data is saved to
% .mat files at the end of the experiment for offline dead reckoning
% analysis.
%
% Sensor data saved:
%   vicon_pose_array      — (x, y, theta) ground truth poses, 3 x N x T
%   imu_orientation_array — fused orientation in degrees, 3 x N x T
%   encoder_array         — wheel tick counts [left; right], 2 x N x T
%   gyro_array            — angular velocity (rad/s), 3 x N x T
%   acc_array             — acceleration (m/s^2), 3 x N x T
%   mag_array             — magnetic field (uT), 3 x N x T

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 5;           % Number of robots
finalCount = 3;  % Number of goal sets to complete before stopping
goal_spacing = 0.5; % Minimum spacing between goal points (meters)

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%% =========================================================
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

% Single-integrator position controller drives robots toward goal positions
single_integrator_position_controller = create_si_position_controller();

% Barrier certificate prevents collisions and boundary violations
si_barrier_cert = create_si_barrier_certificate_with_boundary();

% Maps single-integrator velocities to unicycle commands
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics();

% Checks if all robots have reached their goals (loose — goal-set checker)
all_reached = create_at_position('PositionError', 0.10);

% Checks if an individual robot has reached its goal (tight — zero-velocity checker)
at_goal_checker = create_at_position('PositionError', 0.08);

%% =========================================================
%  GOAL INITIALIZATION
%  =========================================================

% Randomized goal positions within the arena, kept away from the boundary
% by 1.5 robot diameters on each side
arena_width = r.boundaries(2) - r.boundaries(1) - 3*r.robot_diameter;
arena_height = r.boundaries(4) - r.boundaries(3) - 3*r.robot_diameter;

goal_points = generate_random_poses(N, ...
    'Width', arena_width, ...
    'Height', arena_height, ...
    'Spacing', goal_spacing);

%% =========================================================
%  DATA STORAGE INITIALIZATION
%  =========================================================

% Each sensor array grows by one page (3rd dimension) per iteration.
% Pre-allocating is not possible since the number of iterations is not
% known in advance — the loop runs until finalCount goal sets are reached.
vicon_pose_array = [];
imu_orientation_array = [];
encoder_array = [];
gyro_array = [];
mag_array = [];
acc_array = [];

%% =========================================================
%  MAIN EXPERIMENT LOOP
%  =========================================================

% Get initial poses and take one step before entering the loop
x = r.get_poses();
x_si = uni_to_si_states(x);
r.step();

count = 0; % Number of goal sets completed

while true

    % Retrieve the most recent poses from the Robotarium.
    % The time delay is approximately 0.033 seconds.
    x = r.get_poses();
    x_si = uni_to_si_states(x);

    % ---------------------------------------------------------
    % Record all sensor readings for this iteration
    % ---------------------------------------------------------
    vicon_pose_array(:, :, end+1) = x;                    %#ok<SAGROW>
    imu_orientation_array(:, :, end+1) = r.get_orientations(); %#ok<SAGROW>
    encoder_array(:, :, end+1) = r.get_encoders();        %#ok<SAGROW>
    gyro_array(:, :, end+1) = r.get_gyros();              %#ok<SAGROW>
    acc_array(:, :, end+1) = r.get_accelerations();       %#ok<SAGROW>
    mag_array(:, :, end+1) = r.get_magnetic_fields();     %#ok<SAGROW>

    % ---------------------------------------------------------
    % Check if all robots have reached their goal positions.
    % ---------------------------------------------------------
    if all_reached(x_si, goal_points(1:2,:))

        count = count + 1;

        % All goal sets completed — exit the loop
        if count == finalCount
            break;
        end

        % Choose new randomized goal positions for the next set
        goal_points = generate_random_poses(N, ...
            'Width', arena_width, ...
            'Height', arena_height, ...
            'Spacing', goal_spacing);
    end

    % ---------------------------------------------------------
    % Compute control inputs
    % ---------------------------------------------------------

    % Drive all robots toward their goal positions
    dxi = single_integrator_position_controller(x_si, goal_points(1:2, :));

    % Zero velocity for any robot that has already reached its goal,
    % using a tighter position tolerance than the goal-set checker.
    % at_goal_checker returns true if ALL robots are done; we instead
    % check each robot individually against its own column in goal_points.
    for k = 1:N
        if at_goal_checker(x_si(:,k), goal_points(1:2, k))
            dxi(:, k) = zeros(2, 1);
        end
    end

    % Apply barrier certificate to prevent collisions and boundary violations
    dxi = si_barrier_cert(dxi, x_si);

    % Convert single-integrator to unicycle velocities
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

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();