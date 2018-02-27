classdef ARobotarium < handle
    %APIAbstract This is an interface for the Robotarium class that
    %ensures the simulator and the robots match up properly.  You should
    %definitely NOT MODIFY this file.  Also, don't submit this file with
    %your algorithm.

    properties (GetAccess = protected, SetAccess = protected)
        robot_handle
        robot_body

        % Stuff for saving data
        file_path
        current_file_size
        current_saved_iterations
        % Path to mat file to keep this in memory
        mat_file_path
        boundary_patch
        boundary_points = {[-1.5, 1.5, 1.5, -1.5], [-1.5, -1.5, 1.5, 1.5]};
    end

    properties (GetAccess = public, SetAccess = protected)
        % Time step for the Robotarium
        time_step = 0.033
        maxLinearVelocity = 0.1
        maxAngularVelocity = 2*pi
        robot_diameter = 0.08
        number_of_agents
        velocities
        poses
        led_commands
        
        %Saving parameters
        save_data

        % Figure handle for simulator
        figure_handle
        show_figure

        % Arena parameters
        boundaries = [-1.5, 1.5, -1.5, 1.5];
    end

    methods (Abstract)

        %Try this one out...
        % We can use this to finish saving / clean up after MQTT
        call_at_scripts_end(this)

        % Getters
        % Get poses must be implemented independently
        get_poses(this)

        %Update functions
        step(this);
    end

    methods
        function this = ARobotarium(number_of_agents, save_data, show_figure, initial_poses)
            this.number_of_agents = number_of_agents;
            this.save_data = save_data;
            this.show_figure = show_figure;

            this.velocities = zeros(2, number_of_agents);
            this.led_commands = zeros(4, number_of_agents);
            this.poses = initial_poses;

            % If save data, set up the file saving variables
            if(save_data)
                date = datetime('now');
                this.file_path = 'robotarium_data';
                this.file_path = strcat(this.file_path, '_', num2str(date.Month), '_', num2str(date.Day), '_', ...
                num2str(date.Year), '_', num2str(date.Hour), '_', ...
                num2str(date.Minute), '_', num2str(round(date.Second)), '.mat');

                this.current_file_size = 100;
                this.current_saved_iterations = 1;

                % Plus one for timestamp
                robotarium_data = zeros(5*number_of_agents+1, this.current_file_size);
                save(this.file_path, 'robotarium_data', '-v7.3')

                this.mat_file_path = matfile(this.file_path, 'Writable', true);
            end

            if(show_figure)
                this.initialize_visualization()
            end
        end

        function agents = get_number_of_agents(this)
           agents = this.number_of_agents;
        end

        function this = set_velocities(this, ids, vs)
            N = size(vs, 2);

            assert(N<=this.number_of_agents, 'Column size of vs (%i) must be <= to number of agents (%i)', ...
                N, this.number_of_agents);

            % Threshold velocities
            for i = 1:N
                if(abs(vs(1, i)) > this.maxLinearVelocity)
                   vs(1, i) = this.maxLinearVelocity*sign(vs(1,i));
                end

                if(abs(vs(2, i)) > this.maxAngularVelocity)
                   vs(2, i) = this.maxAngularVelocity*sign(vs(2, i));
                end
            end

            this.velocities(:, ids) = vs;
        end
        
        % Commands is [r g b index] x N
        function this = set_leds(this, ids, commands)
            N = size(commands, 2);

            assert(N<=this.number_of_agents, 'Column size of vs (%i) must be <= to number of agents (%i)', ...
                N, this.number_of_agents);
            
            assert(all(all(commands(1:3, :) <= 255)) && all(all(commands(1:3, :) >= 0)), 'RGB commands must be between 0 and 255');
            assert(all(commands(4, :) >= 0) && all(commands(4, :) <= 1), 'LED index must be 0 or 1');                              
            
            % Only set LED commands for the selected robots
            this.led_commands(:, ids) = commands;
        end

        function iters = time2iters(this, time)
           iters = time / this.time_step;
        end
    end

    methods (Access = protected)

        % Initializes visualization of GRITSbots
        function initialize_visualization(this)
            % Initialize variables
            N = this.number_of_agents;
            offset = 0.05;

            % Scale factor (max. value of single Gaussian)
            scaleFactor = 50;
            figPhi = figure;
            this.figure_handle = figPhi;

            % Plot Robotarium boundaries %Maria
            this.boundary_patch = patch('XData', this.boundary_points{1}, 'YData', this.boundary_points{2}, ...
            'FaceColor', 'none', ...
            'LineWidth', 3, ...,
            'EdgeAlpha', 0.5, ...
            'EdgeColor', [0, 0, 0]);

            %plot(im)
            set(figPhi,'color','white');

            % Set axis
            robotPlaneAxes = gca;

            % Limit view to xMin/xMax/yMin/yMax
            axis(robotPlaneAxes, [this.boundaries(1) - offset,this.boundaries(2)+offset,this.boundaries(3)-offset,this.boundaries(4)+offset])
            caxis([0,1.5*scaleFactor])
            set(robotPlaneAxes,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[0.96 1 1])

            % Store axes
            axis(robotPlaneAxes,'off')
            set(robotPlaneAxes,'position',[0 0 1 1],'units','normalized','YDir','normal')

            hold on % "This ride's about to get bumpy!"

            % Let's jump through hoops to make the robot diameter look to data scale
            curUnits = get(robotPlaneAxes, 'Units');
            set(robotPlaneAxes, 'Units', 'Points');
            set(robotPlaneAxes, 'Units', curUnits);

            offset = [-0.1 0.1];
            xlim(this.boundaries(1:2)+offset); ylim(this.boundaries(3:4)+offset);

            % Static legend
            setappdata(gca,'LegendColorbarManualSpace',1);
            setappdata(gca,'LegendColorbarReclaimSpace',1);

            assert(N <= 100, 'Number of robots (%i) must be <= 100', N);

            this.robot_handle = cell(1, N);
            for ii = 1:N
                data = gritsbot_patch;
                this.robot_body = data.vertices;
                x  = this.poses(1, ii);
                y  = this.poses(2, ii);
                th = this.poses(3, ii) - pi/2;
                rotation_matrix = [...
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0 0 1];
                transformed = this.robot_body*rotation_matrix';
                this.robot_handle{ii} = patch(...
                          'Vertices', transformed(:, 1:2), ...
                          'Faces', data.faces, ...
                          'FaceAlpha', 0.6, ...
                          'FaceColor', 'flat', ...
                          'FaceVertexCData', data.colors, ...
                          'EdgeColor','none');
            end
        end

        function draw_robots(this)
            for ii = 1:this.number_of_agents
                x  = this.poses(1, ii);
                y  = this.poses(2, ii);
                th = this.poses(3, ii) - pi/2;
                rotation_matrix = [...
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0 0 1
                ];
                transformed = this.robot_body*rotation_matrix';
                set(this.robot_handle{ii},'Vertices', transformed(:, 1:2));
            end

            if(this.number_of_agents <= 6)
                drawnow
            else
                drawnow limitrate
            end
        end

        function save(this)
            
            this.mat_file_path.robotarium_data(:, this.current_saved_iterations) = ...
                [reshape([this.poses ; this.velocities], [], 1) ; double(tic())];
            
            this.current_saved_iterations = this.current_saved_iterations + 1;
        end
    end
end
