% SENSOR_READINGS
% Demonstrates how to obtain various sensor readings from the GTernal robot
% in the Robotarium simulator. A single robot navigates a serpentine grid
% path through a field of obstacles, recording sensor data at each waypoint.
%
% Sensor data recorded at each goal position:
%   - Poses          — (x, y, theta) in meters and radians, 3 x 1
%   - Distances      — 7 distance sensor readings in meters, 7 x 1
%                      -1 indicates no obstacle detected within range
%   - Accelerations  — (x, y, z) in m/s^2, 3 x 1 (IMU frame)
%   - Magnetic fields — (x, y, z) in uT, 3 x 1 (IMU frame)
%   - Orientations   — fused (roll, pitch, yaw) in degrees, 3 x 1 (IMU frame)
%   - Encoders       — (left, right) wheel ticks since start, 2 x 1
%
% Soobum Kim
% 01/2026

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 1; % Number of robots

%% =========================================================
%  OBSTACLE SETUP
%  =========================================================

% Obstacles are defined as line segments for distance sensor simulation
% only — there is no collision simulation with these obstacles.
%
% Each obstacle is a 2x2 array of two endpoints:
%   [x1, y1;
%    x2, y2]
% Multiple obstacles are stacked along the 1st dimension: M x 2 x 2.
%
% NOTE: The arena boundary is NOT treated as an obstacle for distance
% sensors by default. Add them manually if needed. Arena corners:
%   Bottom-left: (-1.6, -1)   Bottom-right: (1.6, -1)
%   Top-right:   ( 1.6,  1)   Top-left:    (-1.6,  1)
obstacles = zeros(8, 2, 2);
obstacles(1,:,:) = [-1.0, -0.5; -1.0, 0.5];
obstacles(2,:,:) = [-0.6, -0.5; -0.6, 0.5];
obstacles(3,:,:) = [ 0.6, -0.5;  0.6, 0.5];
obstacles(4,:,:) = [ 1.0, -0.5;  1.0, 0.5];
obstacles(5,:,:) = [-1.4, -0.9; -1.4, 0.9];
obstacles(6,:,:) = [-1.4,  0.9;  1.4, 0.9];
obstacles(7,:,:) = [ 1.4,  0.9;  1.4, -0.9];
obstacles(8,:,:) = [ 1.4, -0.9; -1.4, -0.9];

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

initial_conditions = [-1.2; -0.6; 0]; % Initial pose [x; y; theta]

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, ...
               'InitialConditions', initial_conditions, ...
               'UseDistanceSensors', true, 'Obstacles', obstacles, "ShowDistanceRays", true, "ShowDistanceEndpoints", true);

%% =========================================================
%  CONTROLLER SETUP
%  =========================================================

% Single-integrator position controller drives the robot to each waypoint
controller = create_si_position_controller();

% Maps single-integrator velocities to unicycle commands
si_to_uni_dynamics = create_si_to_uni_dynamics();

% Large rotation tolerance so the checker only cares about position
init_checker = create_at_position('PositionError', 0.05);

%% =========================================================
%  GOAL GRID INITIALIZATION
%  =========================================================

% Generate a serpentine grid of goal waypoints covering the arena.
% Even columns are flipped vertically so the robot traces a continuous
% back-and-forth path rather than resetting to the top each column.
resolution_x = 0.4;
resolution_y = 0.4;
x_points = -1.2:resolution_x:1.2;
y_points = -0.6:resolution_y:0.6;
[x_goals, y_goals] = meshgrid(x_points, y_points);
y_goals(:, 2:2:end) = -y_goals(:, 2:2:end); % Serpentine pattern

%% =========================================================
%  DATA STRUCTURE INITIALIZATION
%  =========================================================

% Pre-allocate the first column of each field. Additional columns are
% appended as the robot reaches each waypoint.
data = struct();
data.poses = zeros(3, 1);
data.distances = zeros(7, 1);
data.accelerations = zeros(3, 1);
data.magnetic_fields = zeros(3, 1);
data.orientations = zeros(3, 1);
data.encoders = zeros(2, 1);

%% =========================================================
%  MAIN EXPERIMENT LOOP
%  =========================================================

index = 1; % Current goal index into x_goals / y_goals

while index <= numel(x_goals)

    % Retrieve the most recent poses from the Robotarium.
    % The time delay is approximately 0.033 seconds.
    x = r.get_poses();

    goal = [x_goals(index); y_goals(index)];

    % ---------------------------------------------------------
    % Record sensor data when the robot reaches the current goal
    % ---------------------------------------------------------
    if init_checker(x(1:2,:), goal)

        % --- Distance sensors ---
        % 7 sensors per robot. Max range 1.2 m. -1 = no detection.
        % Sensor orientations in robot frame [x; y; theta] (m, m, rad):
        %   [-0.04,  0.00,  0.04,  0.05,  0.04,  0.00, -0.04]
        %   [ 0.04,  0.06,  0.05,  0.00, -0.05, -0.06, -0.04]
        %   [  pi,  pi/2,  pi/4,  0.00, -pi/4, -pi/2,   -pi ]
        distances = r.get_distances(); % 7 x N

        % --- IMU: accelerometer ---
        % Acceleration in IMU frame (m/s^2), axes relative to robot body:
        %   X: Left,  Y: Backward,  Z: Down
        accelerations = r.get_accelerations(); % 3 x N

        % --- IMU: magnetometer ---
        % Magnetic field in IMU frame (uT), simulated from recorded arena
        % data. Axes relative to robot body:
        %   X: Right,  Y: Forward,  Z: Up
        magnetic_fields = r.get_magnetic_fields(); % 3 x N

        % --- IMU: fused orientation ---
        % Sensor-fused orientation (degrees). Axes are remapped in IMU frame:
        %   Roll -> Yaw,  Pitch -> Roll,  Yaw -> Pitch
        % Orientations are relative to the robot's charging station heading;
        % a robot's 0 deg is either 90 or 180 deg in the global frame.
        % IMU position in robot frame: [0.0575; -0.0130; 0.0] m from axle center.
        orientations = r.get_orientations(); % 3 x N

        % --- Wheel encoders ---
        % Cumulative tick counts since experiment start [left; right].
        % 28 counts/revolution x 100.37 gear ratio ≈ 2810 ticks/wheel revolution.
        % Wheel radius: 0.016 m,  Wheelbase: 0.105 m.
        encoders = r.get_encoders(); % 2 x N

        % Store all readings for this waypoint
        data.poses(:, index) = x;
        data.distances(:, index) = distances;
        data.accelerations(:, index) = accelerations;
        data.magnetic_fields(:, index) = magnetic_fields;
        data.orientations(:, index) = orientations;
        data.encoders(:, index) = encoders;

        index = index + 1;
    end

    % ---------------------------------------------------------
    % Compute and send control inputs
    % ---------------------------------------------------------
    dxi = controller(x(1:2,:), goal);
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