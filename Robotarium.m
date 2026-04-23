classdef Robotarium < ARobotarium
    % Robotarium This object represents your communications with the 
    % GRITSbots.
    %
    % THIS CLASS SHOULD NEVER BE MODIFIED

    properties (GetAccess = private, SetAccess = private)
        checked_poses_already = false % Whether GET_POSES has been checked this iteration
        called_step_already = true % Whether STEP has been called this iteration
        
        iteration = 0; % How many times STEP has been called
        errors = {}; % Accumulated errors for the simulation
    end

    methods
        function this = Robotarium(varargin)
            % ROBOTARIUM Initializes the object.
            % 
            %   ROBOTARIUM('NumberOfRobots', 4) creates a ROBOTARIUM
            %   object with 4 robots.
            %
            %   ROBOTARIUM('NumberOfRobots', 1, 'ShowFigure', false) 
            %   creates a ROBOTARIUM object with 1 robot and shows no 
            %   figure.
            %
            %   Name-Value Parameters:
            %
            %   'NumberOfRobots' (int)
            %       Positive integer specifying the number of robots (N) in the
            %       simulation.
            %
            %   'ShowFigure' (logical)
            %       If true, renders a graphical visualization of the environment.
            %
            %   'FigureHandle' (handle)
            %       Optional figure handle for drawing the simulation.
            %
            %   'InitialConditions' (3 x N double)
            %       Matrix of initial robot poses. Each column is [x; y; theta].
            %       If empty, valid collision-free initial conditions are generated.
            %
            %   'UseDistanceSensors' (logical)
            %       Enables simulation of 7 range sensors per robot.
            %
            %   'Obstacles' (M x 2 x 2 double)
            %       Line-segment obstacles. Each obstacle is:
            %       [x1, y1; x2, y2] representing endpoints in world coordinates.
            %
            %   'SkipInitialization' (logical)skip_initialization: bool, optional
            %       If True, the simulator will skip the initialization phase where
            %       robots drive onto the testbed and directly set the initial conditions.
            %       This will be set to False by default and the verifier will always
            %       use the initialization phase, but setting this to True may be useful
            %       when debugging one's algorithm (default ``False``)
            %
            %   'ShowArenaBoundaries' (logical)
            %       If true, renders the arena boundaries as a box.
            %
            %   'ShowRobotPatches' (logical)
            %       If true, renders the robots as patches instead of points.
            %
            %   'ShowDistanceEndpoints' (logical)
            %       If true, renders the endpoints of distance sensor rays as points.
            %
            %   'ShowDistanceRays' (logical)
            %       If true, renders the distance sensor rays as lines.
            %
            %   'ShowObstacles' (logical)
            %       If true, renders the obstacles as patches.
            %   
            %   Example:
            %       r = Robotarium('NumberOfRobots', 10, 'ShowFigure',
            %       true)
            %
            %   Notes:
            %       The option NumberOfRobots should be a positive integer.
            %       The option ShowFigure should be a boolean value.       
            %       The option InitialConditions should be a 3 x
            %       NumberOfRobots matrix of initial poses.

            % The input will be validated by ARobotarium
            this = this@ARobotarium(varargin{:});

            this.poses = generate_initial_poses(this.number_of_robots, ...
                this.boundaries(2)-this.boundaries(1) - this.robot_diameter, ...
                this.boundaries(4)-this.boundaries(3) - this.robot_diameter);

            initial_conditions = this.initial_conditions;
            if(isempty(initial_conditions))
                initial_conditions = generate_random_poses(this.number_of_robots, ...
                    'Spacing', 0.5, ...
                    'Width', 2.75, ...
                    'Height', 1.75);
            end
            
            assert(all(size(initial_conditions) == [3, this.number_of_robots]), 'Initial conditions must be 3 x %i', this.number_of_robots);            

            if this.skip_initialization
                this.poses = initial_conditions;
            else
                this.initialize(initial_conditions);
            end
            this.initializing = false;
        end

        function initialize(this, initial_conditions)
            % Drive to the initial conditions
            fprintf("Initializing %d robots to initial conditions...\n", this.number_of_robots);

            init_label = [];
            if this.show_figure
                font_size = this.determine_font_size(0.1);
                init_label = text( ...
                    0, -0.5, ...
                    '...INITIALIZING...', ...
                    'FontSize', font_size, ...
                    'Color', 'b', ...
                    'FontWeight', 'bold', ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle');
            end

            barrier   = create_uni_barrier_certificate();
            controller = create_pose_controller_hybrid( ...
                'AngularVelocityLimit', pi / 3.0, ...
                'PositionEpsilon',       0.03, ...
                'PositionError',         0.04, ...
                'RotationError',         0.1  ...
            );
            at_pose = create_at_pose('PositionError', 0.05, 'RotationError', 0.2);

            x = this.get_poses();

            % Deadlock detection state
            step              = 0;
            deadlock_steps    = 5 * 30;
            deadlock_epsilon  = 0.1;
            timers            = zeros(1, this.number_of_robots);
            last_deadlock_poses = x;
            waypointing       = false(1, this.number_of_robots);
            waypoints         = generate_random_poses(this.number_of_robots, ...
                                    'spacing', 0.4, 'width', 2.75, 'height', 1.75);

            this.step();

            [all_at_pose, ~] = at_pose(x, initial_conditions);
            while ~all_at_pose
                x   = this.get_poses();
                dxu = controller(x, initial_conditions);

                [~, initialized_ids] = at_pose(x, initial_conditions);
                all_ids   = 1:this.number_of_robots;
                uninit_ids = setdiff(all_ids, initialized_ids + 1); % +1: 0-indexed -> 1-indexed

                for i = uninit_ids
                    if waypointing(i)
                        % Drive toward waypoint instead of goal
                        dxu(:, i) = controller(x(:, i), waypoints(:, i));

                        if norm(last_deadlock_poses(1:2, i) - x(1:2, i)) > 0.1
                            waypointing(i) = false;
                        elseif step - timers(i) > deadlock_steps
                            waypoints(:, i) = new_random_waypoint(this, x, i);
                            timers(i) = step;
                        end

                    elseif step - timers(i) > deadlock_steps && ...
                        norm(last_deadlock_poses(1:2, i) - x(1:2, i)) < deadlock_epsilon
                        % Robot is deadlocked, assign a waypoint
                        waypointing(i)   = true;
                        waypoints(:, i)  = new_random_waypoint(this, x, i);
                        timers(i)        = step;

                    elseif norm(last_deadlock_poses(1:2, i) - x(1:2, i)) > deadlock_epsilon || ...
                        norm(controller(x(:, i), initial_conditions(:, i))) <= 0.1
                        % Robot is moving or already near goal, reset timer
                        timers(i)              = step;
                        last_deadlock_poses(:, i) = x(:, i);
                    end
                end

                step    = step + 1;
                dxu_safe = barrier(dxu, x);
                this.set_velocities(1:this.number_of_robots, dxu_safe);
                this.step();

                [all_at_pose, ~] = at_pose(x, initial_conditions);
            end

            fprintf("At initial conditions... Starting Experiment\n");
            if ~isempty(init_label)
                delete(init_label);
            end
        end


        % Helper: generate a random escape waypoint near robot i
        function wp = new_random_waypoint(this, x, i)
            wp = [ ...
                this.rand_between(min(x(1,i) + 0.25, this.boundaries(2)), ...
                            max(x(1,i) - 0.25, this.boundaries(1))); ...
                this.rand_between(min(x(2,i) + 0.25, this.boundaries(4)), ...
                            max(x(2,i) - 0.25, this.boundaries(3))); ...
                this.rand_between(-pi, pi) ...
            ];
        end

        function v = rand_between(~, lo, hi)
            v = lo + (hi - lo) * rand();
        end

        function poses = get_poses(this)
            % GET_POSES Returns the current poses of the robots
            %
            %   GET_POSES() returns a 3 x NUMBER_OF_ROBOTS matrix of poses
            %
            %   Example:
            %       x = this.get_poses()
            %
            %   Notes:
            %       This function should only be called once per call of
            %       STEP
            
            assert(~this.checked_poses_already, 'Can only call get_poses() once per call of step()!');

            poses = this.poses;

            %Make sure it's only called once per iteration
            this.checked_poses_already = true;
            this.called_step_already = false;
        end

        function simulate_encoder_readings(this)
            % Simulate encoder readings based on wheel velocities

            % Convert linear and angular velocities to motor angular velocities
            left_motor_angular_velocity  = (2*this.velocities(1,:) - this.base_length*this.velocities(2,:)) / (2*this.wheel_radius);
            right_motor_angular_velocity = (2*this.velocities(1,:) + this.base_length*this.velocities(2,:)) / (2*this.wheel_radius);

            delta_encoder = this.encoder_counts_per_revolution * this.motor_gear_ratio / (2*pi) * [left_motor_angular_velocity; right_motor_angular_velocity] * this.time_step;

            % Per-count noise: error std scales with sqrt(number of counts) in this step
            n_counts = max(abs(delta_encoder), 1.0);
            step_noise_std = this.encoder_noise_std * sqrt(n_counts);
            encoder_noise = step_noise_std .* randn(size(delta_encoder));
            delta_encoder = delta_encoder + encoder_noise;

            this.encoders = this.encoders + round(delta_encoder);
        end

        function simulate_accelerations(this)
            % Simulates the accelerometer readings for the robots based on their current velocities and the physics of the system. This includes:
            %   - Translational acceleration (change in linear velocity)
            %   - Tangential acceleration (due to angular acceleration)
            %   - Centripetal acceleration (due to angular velocity)

            % Additionally, the IMU is oriented such that the X-axis points forward, the Y-axis points left, and the Z-axis points up.

            linear_accelerations  = (this.velocities(1,:) - this.velocities_old(1,:)) / this.time_step; % 1 x N
            angular_accelerations = (this.velocities(2,:) - this.velocities_old(2,:)) / this.time_step; % 1 x N
            omega_z = this.velocities(2,:); % 1 x N

            translational = [linear_accelerations;
                            zeros(1, this.number_of_robots);
                            zeros(1, this.number_of_robots)];

            tangential = [-angular_accelerations * this.imu_orientation(2);
                        angular_accelerations * this.imu_orientation(1);
                        zeros(1, this.number_of_robots)];

            centripetal = [-omega_z.^2 * this.imu_orientation(1);
                        -omega_z.^2 * this.imu_orientation(2);
                        zeros(1, this.number_of_robots)];

            gravity = [zeros(1, this.number_of_robots);
                    zeros(1, this.number_of_robots);
                    -9.81 * ones(1, this.number_of_robots)];

            imu_acc = translational + tangential + centripetal + gravity;

            acc_noise = this.accelerometer_noise_stds .* randn(3, this.number_of_robots);
            this.accelerations = imu_acc + acc_noise;
        end

        function simulate_gyros(this)
            % Simulates the gyrometer readings for the robot based on their current angular velocities.
            % The gyros measure the angular velocity around the Z-axis (yaw rate) with some added noise.

            this.gyros = [randn(1, this.number_of_robots) * this.gyro_noise_stds(1);       % X-axis gyro noise
                        randn(1, this.number_of_robots) * this.gyro_noise_stds(2);         % Y-axis gyro noise
                        this.velocities(2,:) + randn(1, this.number_of_robots) * this.gyro_noise_stds(3)];      % Z-axis gyro measurement with noise
        end

        function simulate_magnetometers(this)
            % Simulates the magnetometer readings for each robot based on their orientation in the testbed.
            % Note: in the testbed, magnetic fields are corrected such that magnetic north is the x-axis of the testbed.

            mag_noise = this.magnetometer_noise_stds .* randn(3, this.number_of_robots);
            this.magnetic_fields = ...
                [this.magnetometer_xy_avg * cos(this.poses(3,:))  + mag_noise(1,:);
                -this.magnetometer_xy_avg * sin(this.poses(3,:))  + mag_noise(2,:);
                this.magnetometer_z_avg  * ones(1, this.number_of_robots) + mag_noise(3,:)];
        end

        function simulate_orientations(this)
            % Simulates the orientation readings for each robot based on their current pose and some added noise.
            % Note: On the robotarium, a sensor fusion algorithm is run on the IMU data to produce a stable orientation estimate.

            orientation_degrees = this.poses(3,:) * (180/pi);
            this.orientations = mod(orientation_degrees + this.orientation_noise_std * randn(1, this.number_of_robots), 360);
        end

        function step(this)
            % STEP Steps the simulation, updating poses of robots and
            % checking for errors
            %
            %   STEP()
            %
            %   Example:
            %       object.step()
            %
            %   Notes:
            %       Should be called everytime GET_POSES is called
            
            assert(~this.called_step_already, 'Make sure you call get_poses before calling step!');

            %Vectorize update to states
            i = 1:this.number_of_robots;
                        
            % Validate before thresholding velocities
            es = this.validate();
            this.errors = [this.errors, es];
            this.iteration = this.iteration + 1;
            
            this.velocities = this.threshold(this.velocities);

            %Update velocities using unicycle dynamics
            temp = this.time_step.*this.velocities(1, i);
            this.poses(1, i) = this.poses(1, i) + temp.*cos(this.poses(3, i));
            this.poses(2, i) = this.poses(2, i) + temp.*sin(this.poses(3, i));
            this.poses(3, i) = this.poses(3, i) + this.time_step.*this.velocities(2, i);

            %Ensure that the orientations are in the right range
            this.poses(3, i) = atan2(sin(this.poses(3, i)), cos(this.poses(3, i)));

            % Update distance sensors if enabled
            if this.distance_sensors_enabled
                this.distances = simulate_distance_sensors(...
                    this.poses, ...
                    this.obstacles, ...
                    this.distance_sensors_orientation, ...
                    this.robot_diameter, ...
                    this.center_offset, ...
                    this.distance_sensor_range, ...
                    this.distance_sensor_error, ...
                    this.distance_sensor_dropout_prob, ...
                    this.distance_sensor_outlier_prob);
                this.distance_end_points = this.calculate_endpoints(this.distances);
            end

            % Update encoder readings
            this.simulate_encoder_readings();

            % Update IMU measurements
            this.simulate_accelerations();
            this.simulate_gyros();
            this.simulate_magnetometers();
            this.simulate_orientations();

            %Allow getting of poses again
            this.checked_poses_already = false;
            this.called_step_already = true;            
            
            if(this.show_figure)
                this.draw_robots();
                if ~this.initializing && this.distance_sensors_enabled && this.show_distance_endpoints
                    this.draw_distance_endpoints(this.distance_end_points);
                end
                if ~this.initializing && this.distance_sensors_enabled && this.show_distance_rays
                    this.draw_distance_rays(this.distance_end_points);
                end
                uistack([this.robot_handle{:}],'top');
            end
            
            % Save old velocities to compute acceleration properly.
            this.velocities_old = this.velocities;
        end
        
        function debug(this)
            num_errors = 3;
            count = zeros(1, num_errors);
            for i = 1:numel(this.errors)
                count(this.errors{i}) = count(this.errors{i}) + 1;
            end
 
            fprintf('Your simulation took approximately %.2f real seconds to execute.\n\n', this.iteration*this.time_step);
 
            % Hard errors (will cause experiment rejection)
            hard_error_strings = cell(1, num_errors);
            hard_error_strings{RobotariumError.RobotsTooClose}          = 'time steps where robots were too close (potential collision)';
            hard_error_strings{RobotariumError.RobotsOutsideBoundaries} = 'time steps where robots were outside boundaries';

            % Soft errors (will not cause experiment rejection)

            actuator_count = count(RobotariumError.ExceededActuatorLimits);
 
            % Overall verdict
            hard_errors = count(RobotariumError.RobotsTooClose) + count(RobotariumError.RobotsOutsideBoundaries);
            if hard_errors == 0 && actuator_count == 0
                fprintf('No errors or warnings in your simulation!  Your script will run on the Robotarium!\n\n');
            elseif hard_errors == 0
                if actuator_count > 0
                    fprintf(['No hard errors detected, your script will run on the Robotarium! \n\n']);
                    fprintf(['WARNING: %i out of %i (~ %.0f%%) timesteps had velocity command(s) exceed actuator limits.\n', ...
                            '         A large percentage of exceeded actuator commands will cause noticeable differences\n', ...
                            '         between this simulation and what is performed on the physical Robotarium.\n'], ...
                            actuator_count, this.iteration, actuator_count/this.iteration*100);
                end
            else
                fprintf('Errors detected, your script will NOT run on the Robotarium! \n\n');
                fprintf('Error: \n');
                print_error = @(x) fprintf('\t Simulation had %i %s.\n', count(x), hard_error_strings{x});
                print_error(RobotariumError.RobotsTooClose);
                print_error(RobotariumError.RobotsOutsideBoundaries);

                fprintf('\nPlease fix the noted errors in your simulation; otherwise, your experiment will be rejected.\n');
            end
        end
    end
end
