classdef ARobotarium < handle
    % AROBOTARIUM This is an interface for the Robotarium class that
    % ensures the simulator and the robots match up properly.  You should
    % definitely NOT MODIFY this file.  Also, don't submit this file.
    
    properties (GetAccess = protected, SetAccess = protected)
        robot_handle % Handle for the robots patch objects
        robot_body  % Base robot body position used for rendering
        
        boundary_patch % Path to denote the Robotarium's boundary

        distance_endpoint_patches % Handle for the distance sensor endpoints rendering
        distance_ray_lines % Handle for the distance sensor rays
    end
    
    properties (Constant)
        time_step = 0.033;                          % seconds
        max_linear_velocity = 0.2;                  % m/s 
        robot_diameter = 0.11;                      % m
        wheel_radius = 0.016;                       % m
        base_length = 0.11;                        % m
        collision_diameter = 0.135;                 % m
        collision_offset = 0.025;                   % m
        boundaries = [-1.6, 1.6, -1, 1];            % m [x_min, x_max, y_min, y_max]
        center_offset = 0.01                        % m (Offset forwards where a robot's collider is placed)
        distance_sensor_error = 0.03;               % fractional (3% error)
        distance_sensor_dropout_prob = 0.035;       % probability [0, 1]
        distance_sensor_outlier_prob = .014;        % probability [0, 1] 
        distance_sensor_range = 1.2;                % m 
        distance_sensors_orientation = [-0.04, 0.0,  0.04, 0.05, 0.04,   0.0   -0.04;  % m (x, y) and rad (theta) -> [x; y; theta] x 7
                                         0.04, 0.06, 0.05, 0.0,  -0.05, -0.06, -0.04;
                                         pi,   pi/2, pi/4, 0.0,  -pi/4, -pi/2, -pi];

        encoder_counts_per_revolution = 28;                             % counts/revolution
        motor_gear_ratio = 100.37;                                                     
        imu_orientation = [0.0594 - 0.00319; 0.0344628 - 0.0475; 0.0];  % m (x, y offset from axle center) and rad (heading of IMU in robot frame) -> [x, y, theta]
        
        encoder_noise_std        = 0.25;                             % counts
        accelerometer_noise_stds = [0.012929; 0.012127; 0.052979];   % m/s^2 [x; y; z]
        gyro_noise_stds          = [0.001663; 0.001216; 0.002372];   % rad/s [x; y; z]
        magnetometer_noise_stds  = [3.275843; 2.365798; 5.232685];   % uT [x; y; z]
        magnetometer_z_avg       = -39.8594;                         % uT
        magnetometer_xy_avg      = 6.0712;                           % uT
        orientation_noise_std    = 0.310830;                         % degrees

        tab20 = [
            0.1216, 0.4667, 0.7059; 0.6824, 0.7804, 0.9098; % Blues
            1.0000, 0.4980, 0.0549; 1.0000, 0.7333, 0.4706; % Oranges
            0.1725, 0.6275, 0.1725; 0.5961, 0.8745, 0.5412; % Greens
            0.8392, 0.1529, 0.1569; 1.0000, 0.5961, 0.5882; % Reds
            0.5804, 0.4039, 0.7412; 0.7725, 0.6902, 0.8353; % Purples
            0.5490, 0.3373, 0.2941; 0.7686, 0.6118, 0.5804; % Browns
            0.8902, 0.4667, 0.7608; 0.9686, 0.7137, 0.8235; % Pinks
            0.4980, 0.4980, 0.4980; 0.7804, 0.7804, 0.7804; % Grays
            0.7373, 0.7412, 0.1333; 0.8588, 0.8588, 0.5529; % Olive/Yellow
            0.0902, 0.7451, 0.8118; 0.6196, 0.8549, 0.8980  % Cyans
        ];
    end
    
    properties (GetAccess = public, SetAccess = protected)
        % Maximum wheel velocity of the robots (m/s)
        max_wheel_velocity = ARobotarium.max_linear_velocity/ARobotarium.wheel_radius;
        
        % Maximum angular velocity of the robots (rad/s)
        max_angular_velocity = ...
        2*(ARobotarium.wheel_radius/ARobotarium.robot_diameter) ...
        *(ARobotarium.max_linear_velocity/ARobotarium.wheel_radius);
    
        number_of_robots
        figure_handle

        % Array of line-segment obstacles in Robotarium coordinates -> [m], 2x2xM
        % Each is defined by two endpoints -> [:, 1, :] = start, [:, 2, :] = end
        obstacles
    end
    
    properties (GetAccess = protected, SetAccess = protected)
        % The linear and angular (v, w) velocity of each robot -> [m/s; rad/s], 2xN
        velocities
        % The (x, y, theta) pose of each robot -> [m; m; rad], 3xN
        poses
        % The distance sensor measurements for each robot -> [m], 7xN
        distances
        % The (x, y, z) acceleration of each robot -> [m/s^2], 3xN
        accelerations
        % The (x, y, z) orientation of each robot -> [deg], 1xN
        orientations
        % The (x, y, z) magnetometer measurements of each robot -> [uT], 3xN
        magnetic_fields
        % The (x, y, z) gyro measurements of each robot -> [rad/s], 3xN
        gyros
        % The initial encoder counts -> [counts], 2xN
        initial_encoders
        % The left and right encoder counts of each robot -> [counts], 2xN
        encoders
        % Are the distance sensors enabled (logical)
        distance_sensors_enabled
        % The initial conditions of the robotarium
        initial_conditions

        % The led connected to the robot -> [R; G; B], 3xN
        leds

        % Distance sensor end points (x, y) in Robotarium coordinates -> [m], 2 x 7N
        distance_end_points

        % Previous robot velocities for acceleration calculation -> [m/s; rad/s], 2xN
        velocities_old
        
        % Figure handle for simulator
        show_figure

        % Is the initialization routine being run
        initializing = true;
        % Should we skip the initialization in the simulator
        skip_initialization = false;

        % Should the obstacles be displayed
        show_obstacles = true;
        % Shold the arena boundaries be displayed
        show_arena_boundaries = true;
        % Should the robot patches be displayed
        show_robot_patches = true;
        % Should the distance sensor endpoints be displayed
        show_distance_endpoints = true;
        % Should the distance sensor rays be displayed
        show_distance_rays = false;
    end   
    
    methods (Abstract)                
        % Getters
        get_poses(this)
        
        % Initialization
        initialize(this, initial_conditions)
        
        %Update functions
        step(this);
        debug(this);
    end
    
    methods
        function this = ARobotarium(varargin)
            parser = inputParser();
            parser.KeepUnmatched = true;
            parser.addParameter('NumberOfRobots', -1);
            parser.addParameter('ShowFigure', true);
            parser.addParameter('FigureHandle', []);
            parser.addParameter('InitialConditions', []);
            parser.addParameter('UseDistanceSensors', false);
            parser.addParameter('Obstacles', []); % Default no obstacles
            parser.addParameter('SkipInitialization', false);
            parser.addParameter('ShowArenaBoundaries', true);
            parser.addParameter('ShowRobotPatches', true);
            parser.addParameter('ShowDistanceEndpoints', true);
            parser.addParameter('ShowDistanceRays', false);
            parser.addParameter('ShowObstacles', true);
            parse(parser, varargin{:});

            assert(parser.Results.NumberOfRobots >= 0 && parser.Results.NumberOfRobots <= 50, ...
            'Number of robots (%i) must be >= 0 and <= 50', parser.Results.NumberOfRobots);
            this.number_of_robots = parser.Results.NumberOfRobots;
            N = parser.Results.NumberOfRobots;

            this.velocities = zeros(2, N);
            this.velocities_old = zeros(2, N);
            this.poses = zeros(3, N);
            this.distances = NaN(7, N);
            this.accelerations = zeros(3, N);
            this.orientations = zeros(1, N);
            this.magnetic_fields = zeros(3, N);
            this.initial_encoders = zeros(2, N);
            this.encoders = zeros(2, N);
            this.show_figure = parser.Results.ShowFigure;
            this.leds = zeros(3, N);
            this.initial_conditions = parser.Results.InitialConditions;

            this.skip_initialization = parser.Results.SkipInitialization;
            this.show_arena_boundaries = parser.Results.ShowArenaBoundaries;
            this.show_robot_patches = parser.Results.ShowRobotPatches;
            this.show_distance_endpoints = parser.Results.ShowDistanceEndpoints;
            this.show_distance_rays = parser.Results.ShowDistanceRays;
            this.show_obstacles = parser.Results.ShowObstacles;

            this.distance_sensors_enabled = parser.Results.UseDistanceSensors;
            if this.distance_sensors_enabled
                this.distance_end_points = NaN(2, 7*this.number_of_robots);
            end

            this.obstacles = parser.Results.Obstacles;

            if(parser.Results.ShowFigure)  
                if(isempty(parser.Results.FigureHandle))
                    this.figure_handle = figure();
                else
                    this.figure_handle = parser.Results.FigureHandle;
                end
                
                this.initialize_visualization();
            end                                                  
        end

        function call_at_scripts_end(~)
        end
        
        function agents = get_number_of_robots(this)
            agents = this.number_of_robots;
        end
        
        function this = set_velocities(this, ids, vs)
            N = size(vs, 2);
            
            assert(N<=this.number_of_robots, 'Row size of velocities (%i) must be <= to number of agents (%i)', ...
                N, this.number_of_robots);           
            
            this.velocities(:, ids) = vs;
        end

        function distances = get_distances(this)
            % Get the distance sensor readings for each of the robots

            distances = this.distances;
        end

        function global_distance_readings = get_distance_endpoints(this)
            % Get the transformed end points in global Robotarium coordinates from distance sensor readings

            global_distance_readings = this.distance_end_points;
        end

        function accelerations = get_accelerations(this)
            % Get the acceleration readings for each of the robots

            accelerations = this.accelerations;
        end

        function orientations = get_orientations(this)
            % Get the orientation readings for each of the robots

            orientations = this.orientations;
        end

        function magnetic_fields = get_magnetic_fields(this)
            % Get the magnitude of the magnetic fields for each of the robots

            magnetic_fields = this.magnetic_fields;
        end

        function gyros = get_gyros(this)
            % Get the gyro of each robot

            gyros = this.gyros;
        end

        function encoders = get_encoders(this)
            % Get the encoder values for each of the robots

            encoders = int32(this.encoders) - int32(this.initial_encoders);
        end
        
        function iters = time2iters(this, time)
            iters = time / this.time_step;
        end
    end
    
    methods (Access = protected)

        function font_size = determine_font_size(this, font_height_meters)
            % Font size in points matching a physical height in metres.
            fig = this.figure_handle;

            fig_pos_px = get(fig, 'Position');   % [x, y, width, height] in pixels
            fig_h_px   = fig_pos_px(4);

            arena_h = this.boundaries(4) - this.boundaries(3);

            font_size = fig_h_px * (font_height_meters / arena_h);
        end
        
        function dxu = threshold(this, dxu)
            dxdd = this.uni_to_diff(dxu);
            
            to_thresh = abs(dxdd) > this.max_wheel_velocity;
            dxdd(to_thresh) = this.max_wheel_velocity*sign(dxdd(to_thresh));

            dxu = this.diff_to_uni(dxdd);
        end
        
        function dxdd = uni_to_diff(this, dxu)
            r = this.wheel_radius;
            l = this.base_length;
            dxdd = [
                (1/(2*r))*(2*dxu(1, :) - l*dxu(2, :)) ; ...
                (1/(2*r))*(2*dxu(1, :) + l*dxu(2, :))
                ];
        end
        
        function dxu = diff_to_uni(this, dxdd)
            r = this.wheel_radius;
            l = this.base_length;
            dxu = [
                r/2*(dxdd(1, :) + dxdd(2, :));
                r/l*(dxdd(2, :) - dxdd(1, :))
                ];
        end
        
        function errors = validate(this)
            % VALIDATE meant to be called on each iteration of STEP. 
            % Checks that robots are operating normally.
            
            p = this.poses;
            b = this.boundaries;
            N = this.number_of_robots;
            errors = {};

            if ~this.initializing
                for i = 1:N
                    x = p(1, i);
                    y = p(2, i);
                    
                    if(x < b(1) || x > b(2) || y < b(3) || y > b(4))                   
                        errors{end+1} = RobotariumError.RobotsOutsideBoundaries;
                    end
                end
                
                % Collision checking:
                for i = 1:(N-1)
                    for j = i+1:N     
                        if(norm((p(1:2, i) + ARobotarium.collision_offset*[cos(p(3, i)); sin(p(3,i))]) - (p(1:2, j) + ARobotarium.collision_offset*[cos(p(3, j)); sin(p(3,j))])) <= ARobotarium.collision_diameter)
                            errors{end+1} = RobotariumError.RobotsTooClose;
                        end
                    end
                end
                
                dxdd = this.uni_to_diff(this.velocities);
                exceeding = abs(dxdd) > this.max_wheel_velocity;
                if(any(any(exceeding)))
                    errors{end+1} = RobotariumError.ExceededActuatorLimits;
                end
            end
        end
    end
    
    % Visualization methods
    methods (Access = protected)
               
        % Initializes visualization of GTERNALs
        function initialize_visualization(this)
            N = this.number_of_robots;
            offset = 0.05;
            fig = this.figure_handle;
        
            % Create axes explicitly first so every graphics object
            % can be parented to it  prevents newer MATLAB from
            % creating a second axes on implicit patch/scatter calls
            ax = axes(fig);
            set(fig, 'color', 'white');
        
            % Configure axes appearance and limits
            axis(ax, [this.boundaries(1)-offset, this.boundaries(2)+offset, ...
                      this.boundaries(3)-offset, this.boundaries(4)+offset]);
            set(ax, 'PlotBoxAspectRatio', [1 1 1], 'DataAspectRatio', [1 1 1]);
            axis(ax, 'off');
            setappdata(ax, 'LegendColorbarManualSpace', 1);
            setappdata(ax, 'LegendColorbarReclaimSpace', 1);
        
            % Plot Robotarium boundary  explicit Parent ax
            if this.show_figure && this.show_arena_boundaries
                b = this.boundaries;
                boundary_points = {[b(1) b(2) b(2) b(1)], [b(3) b(3) b(4) b(4)]};
                this.boundary_patch = patch(ax, ...
                    'XData', boundary_points{1}, ...
                    'YData', boundary_points{2}, ...
                    'FaceColor', 'none', ...
                    'LineWidth', 3, ...
                    'EdgeColor', [0, 0, 0]);
            end
        
            % Plot robot bodies  explicit Parent ax
            if this.show_figure && this.show_robot_patches
                this.robot_handle = cell(1, N);
                data = gternal_patch;
                this.robot_body = data.vertices;
                for i = 1:N
                    x  = this.poses(1, i);
                    y  = this.poses(2, i);
                    th = this.poses(3, i) - pi/2;
                    rotation_matrix = [cos(th) -sin(th) x;
                                    sin(th)  cos(th) y;
                                    0        0       1];
                    transformed = this.robot_body * rotation_matrix';
                    this.robot_handle{i} = patch(ax, ...
                        'Vertices', transformed(:, 1:2), ...
                        'Faces', data.faces, ...
                        'FaceColor', 'flat', ...
                        'FaceVertexCData', data.colors, ...
                        'EdgeColor', 'none');
                end
            end
        
            % MATLAB R2022a+ changed scatter() to internally call newplot(ax) even
            % when given an explicit axes handle. newplot() checks the axes
            % NextPlot property, and freshly created axes default to 'replace',
            % which clears all existing children  deleting the robot patch handles
            % created above. Setting NextPlot to 'add' before calling scatter()
            % prevents this.
            set(ax, 'NextPlot', 'add');
            
            if this.show_figure && this.distance_sensors_enabled && this.show_distance_endpoints
                this.distance_endpoint_patches = [];
                for i = 1:N
                    this.distance_endpoint_patches = [this.distance_endpoint_patches, scatter(ax, nan(1, 7), nan(1, 7), 50, this.tab20(i, :), 'filled')];
                end
            end

            if this.show_figure && this.distance_sensors_enabled && this.show_distance_rays
                this.distance_ray_lines = [];
                for i= 1:N
                    ray_lines = [];
                    for j = 1:7
                        ray_lines = [ray_lines, line(ax, ...
                            'XData', nan(1, 2), ...
                            'YData', nan(1, 2), ...
                            'LineWidth', 0.5, ...
                            'Color', this.tab20(i, :))];
                    end
                    this.distance_ray_lines = [this.distance_ray_lines; ray_lines];
                end
            end

            if this.show_figure && this.show_obstacles && ~isempty(this.obstacles)
                for k = 1:size(this.obstacles, 1)
                    line(ax, ...
                        'XData', [this.obstacles(k, 1, 1), this.obstacles(k, 2, 1)], ...
                        'YData', [this.obstacles(k, 1, 2), this.obstacles(k, 2, 2)], ...
                        'LineWidth', 4, ...
                        'Color', [0.5, 0.5, 0.5]);
                end
            end
        
            % Keep hold active so user plotting never resets the axes
            hold(ax, 'on');
        end
        
        function draw_robots(this)
            if this.show_robot_patches
                for i = 1:this.number_of_robots
                    x  = this.poses(1, i);
                    y  = this.poses(2, i);
                    th = this.poses(3, i) - pi/2;
                    rotation_matrix = [...
                        cos(th) -sin(th) x;
                        sin(th)  cos(th) y;
                        0 0 1
                        ];
                    transformed = this.robot_body*rotation_matrix';
                    set(this.robot_handle{i}, 'Vertices', transformed(:, 1:2));
                    
                    % Set LEDs
                    led_values = this.leds / 255;
                    this.robot_handle{i}.FaceVertexCData(4, :) = led_values(:, i);
                end
            end
            drawnow limitrate
        end

        function endpoints = calculate_endpoints(this, distances)
            endpoints = nan(2, 7, this.number_of_robots);
            for i = 1:this.number_of_robots
                R = [cos(this.poses(3, i)), -sin(this.poses(3, i)); sin(this.poses(3, i)), cos(this.poses(3, i))];
                for j = 1:7
                    if distances(j, i) >= 0
                        start_point = R * ARobotarium.distance_sensors_orientation(1:2, j) + this.poses(1:2, i);
                        end_point = start_point + distances(j, i) * [cos(this.poses(3, i) + ARobotarium.distance_sensors_orientation(3, j)); sin(this.poses(3, i) + ARobotarium.distance_sensors_orientation(3, j))];
                        endpoints(:, j, i) = end_point;
                    else
                        endpoints(:, j, i) = [NaN; NaN];
                    end
                end
            end
        end

        function draw_distance_rays(this, endpoints)
            for i = 1:this.number_of_robots
                R = [cos(this.poses(3, i)), -sin(this.poses(3, i)); sin(this.poses(3, i)), cos(this.poses(3, i))];
                for j = 1:7
                    start_point = R * ARobotarium.distance_sensors_orientation(1:2, j) + this.poses(1:2, i);
                    set(this.distance_ray_lines(i, j), 'XData', [start_point(1), endpoints(1, j, i)], 'YData', [start_point(2), endpoints(2, j, i)]);
                end
            end
        end

        function draw_distance_endpoints(this, endpoints)
            for i = 1:this.number_of_robots
                set(this.distance_endpoint_patches(i), 'XData', endpoints(1, :, i), 'YData', endpoints(2, :, i));
            end
        end
    end
end