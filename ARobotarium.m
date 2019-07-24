classdef ARobotarium < handle
    % AROBOTARIUM This is an interface for the Robotarium class that
    % ensures the simulator and the robots match up properly.  You should
    % definitely NOT MODIFY this file.  Also, don't submit this file.
    
    properties (GetAccess = protected, SetAccess = protected)
        robot_handle % Handle for the robots patch objects
        robot_body  % Base robot body position used for rendering
        
        boundary_patch % Path to denote the Robotarium's boundary
    end
    
    properties (Constant)
        time_step = 0.033
        max_linear_velocity = 0.2  
        robot_diameter = 0.11
        wheel_radius = 0.016;
        base_length = 0.105;          
        boundaries = [-1.6, 1.6, -1, 1];      
    end
    
    properties (GetAccess = public, SetAccess = protected)
        % Maximum wheel velocitry of the robots
        max_wheel_velocity = ARobotarium.max_linear_velocity/ARobotarium.wheel_radius;
        
        max_angular_velocity = ...
        2*(ARobotarium.wheel_radius/ARobotarium.robot_diameter) ...
        *(ARobotarium.max_linear_velocity/ARobotarium.wheel_radius);
    
        number_of_robots
        figure_handle
    end
    
    properties (GetAccess = protected, SetAccess = protected)
        
        
        velocities
        poses
        left_leds
        right_leds                
        % Figure handle for simulator
        
        show_figure
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
        function this = ARobotarium(number_of_robots, show_figure, figure_handle)
            
            assert(number_of_robots >= 0 && number_of_robots <= 50, ...
            'Number of robots (%i) must be >= 0 and <= 50', number_of_robots);
            this.number_of_robots = number_of_robots;
            N = number_of_robots;            
            
            this.poses = zeros(3, N);
            this.show_figure = show_figure;
            this.velocities = zeros(2, N);
            this.left_leds = zeros(3, N);
            this.right_leds = zeros(3, N);    
            
            if(show_figure)  
                if(isempty(figure_handle))
                    this.figure_handle = figure();
                else
                    this.figure_handle = figure_handle;
                end
                
                this.initialize_visualization();
            end                                                  
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
        
        function this = set_left_leds(this, ids, rgbs)
            N = size(rgbs, 2);
            
            assert(N<=this.number_of_robots, 'Row size of rgb values (%i) must be <= to number of agents (%i)', ...
                N, this.number_of_robots);
            
            assert(all(all(rgbs(1:3, :) <= 255)) && all(all(rgbs(1:3, :) >= 0)), 'RGB commands must be between 0 and 255');
            
            % Only set LED commands for the selected robots
            this.left_leds(:, ids) = rgbs;
        end
        
        function this = set_right_leds(this, ids, rgbs)
            N = size(rgbs, 2);
            
            assert(N<=this.number_of_robots, 'Row size of rgb values (%i) must be <= to number of agents (%i)', ...
                N, this.number_of_robots);
            
            assert(all(all(rgbs(1:3, :) <= 255)) && all(all(rgbs(1:3, :) >= 0)), 'RGB commands must be between 0 and 255');
            
            % Only set LED commands for the selected robots
            this.right_leds(:, ids) = rgbs;
        end
        
        function iters = time2iters(this, time)
            iters = time / this.time_step;
        end
    end
    
    methods (Access = protected)    
        
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
           
           for i = 1:N
               x = p(1, i);
               y = p(2, i);
               
               if(x < b(1) || x > b(2) || y < b(3) || y > b(4))                   
                   errors{end+1} = RobotariumError.RobotsOutsideBoundaries;
               end
           end
           
           for i = 1:(N-1)
              for j = i+1:N     
                  if(norm(p(1:2, i) - p(1:2, j)) <= ARobotarium.robot_diameter)
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
    
    % Visualization methods
    methods (Access = protected)
               
        % Initializes visualization of GRITSbots
        function initialize_visualization(this)
            % Initialize variables
            N = this.number_of_robots;
            offset = 0.05;
            
%             fig = figure;
%             this.figure_handle = fig;
            fig = this.figure_handle;
            
            % Plot Robotarium boundaries
            b = this.boundaries;
            boundary_points = {[b(1) b(2) b(2) b(1)], [b(3) b(3) b(4) b(4)]};
            this.boundary_patch = patch('XData', boundary_points{1}, ...
                'YData', boundary_points{2}, ...
                'FaceColor', 'none', ...
                'LineWidth', 3, ...,
                'EdgeColor', [0, 0, 0]);
            
            set(fig, 'color', 'white');
            
            % Set axis
            ax = fig.CurrentAxes;
            
            % Limit view to xMin/xMax/yMin/yMax
            axis(ax, [this.boundaries(1)-offset, this.boundaries(2)+offset, this.boundaries(3)-offset, this.boundaries(4)+offset])
            set(ax, 'PlotBoxAspectRatio', [1 1 1], 'DataAspectRatio', [1 1 1])
            
            % Store axes
            axis(ax, 'off')            
            
            % Static legend
            setappdata(ax, 'LegendColorbarManualSpace', 1);
            setappdata(ax, 'LegendColorbarReclaimSpace', 1);           
            
            % Apparently, this statement is necessary to avoid issues with
            % axes reappearing.
            hold on
            
            this.robot_handle = cell(1, N);
            for i = 1:N
                data = gritsbot_patch;
                this.robot_body = data.vertices;
                x  = this.poses(1, i);
                y  = this.poses(2, i);
                th = this.poses(3, i) - pi/2;
                rotation_matrix = [
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0 0 1];
                transformed = this.robot_body*rotation_matrix';
                this.robot_handle{i} = patch(...
                    'Vertices', transformed(:, 1:2), ...
                    'Faces', data.faces, ...
                    'FaceColor', 'flat', ...
                    'FaceVertexCData', data.colors, ...
                    'EdgeColor','none');
            end
        end
        
        function draw_robots(this)
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
                left = this.left_leds/255;
                right = this.right_leds/255;
            
                this.robot_handle{i}.FaceVertexCData(4, :) = left(:, i);
                this.robot_handle{i}.FaceVertexCData(5, :) = right(:, i);
            end

            drawnow limitrate
        end 
    end
end
