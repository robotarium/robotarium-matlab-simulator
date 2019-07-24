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
            %   Example:
            %       r = Robotarium('NumberOfRobots', 10, 'ShowFigure',
            %       true)
            %
            %   Notes:
            %       The option NumberOfRobots should be a positive integer.
            %       The option ShowFigure should be a boolean value.       
            %       The option InitialConditions should be a 3 x
            %       NumberOfRobots matrix of initial poses.
            
            parser = inputParser;
            
            parser.addParameter('NumberOfRobots', -1);
            parser.addParameter('ShowFigure', true);
            parser.addParameter('FigureHandle', []);
            parser.addParameter('InitialConditions', []);
                        
            parse(parser, varargin{:})
            
            % The input will be validated by ARobotarium
            this = this@ARobotarium(parser.Results.NumberOfRobots, ...
                parser.Results.ShowFigure, parser.Results.FigureHandle);
            
            initial_conditions = parser.Results.InitialConditions;
            
            if(isempty(initial_conditions))
                initial_conditions = generate_initial_conditions(this.number_of_robots, ...
                    'Spacing', 1.5*this.robot_diameter, ...
                    'Width', this.boundaries(2)-this.boundaries(1)-this.robot_diameter, ...
                    'Height', this.boundaries(4)-this.boundaries(3))-this.robot_diameter;
            end
            
            assert(all(size(initial_conditions) == [3, this.number_of_robots]), 'Initial conditions must be 3 x %i', this.number_of_robots);            
            
            % Call initialize during initialization
            this.initialize(initial_conditions);
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
        
        function initialize(this, initial_conditions)
            this.poses = initial_conditions;
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

            %Allow getting of poses again
            this.checked_poses_already = false;
            this.called_step_already = true;            
            
            if(this.show_figure)
                this.draw_robots();
                uistack([this.robot_handle{:}],'top');
            end            
        end
        
        function debug(this)
            num_errors = 3;
            count = zeros(1, num_errors);
            for i = 1:numel(this.errors)
              count(this.errors{i}) = count(this.errors{i}) + 1;                 
            end
            
            fprintf('Your simulation took approximately %.2f real seconds.\n', this.iteration*this.time_step);
            
            error_strings = cell(1, num_errors);
            error_strings{RobotariumError.RobotsTooClose} = 'robots were too close';
            error_strings{RobotariumError.RobotsOutsideBoundaries} = 'robots were outside boundaries'; 
            error_strings{RobotariumError.ExceededActuatorLimits} = 'robots exceeded actuator limits';            
            
            fprintf('Error count for current simulation:\n');
            print_error = @(x) fprintf('\t Simulation had %i %s errors.\n', count(x), error_strings{x});            
            print_error(RobotariumError.RobotsTooClose)
            print_error(RobotariumError.RobotsOutsideBoundaries);
            print_error(RobotariumError.ExceededActuatorLimits);
            
            if(isempty(this.errors))
                fprintf('No errors in your simulation!  Acceptance of experiment likely.\n')               
            else
                fprintf('Please fix the noted errors in your simulation; otherwise, your experiment may be rejected.\n');
            end            
        end
    end
end
