classdef RobotariumBuilder < APIBuilder
    %ROBOTARIUM Summary of this class goes here
    %   Detailed explanation goes here
    
    % Gets properties from abstract class as well.
    properties
        boundaries = [-0.6, 0.6, -0.35, 0.35]; 
        robot_diameter = 0.08
    end
    
    methods
        
        function this = RobotariumBuilder()
          this.available_agents = randi(10); 
        end
        
        function number_of_agents = get_available_agents(this)
           number_of_agents = this.available_agents;
        end
        
        function robotarium_obj = build(this)
            arena_width = this.boundaries(2) - this.boundaries(1);
            arena_height = this.boundaries(4) - this.boundaries(3);
            
            numX = floor(arena_width / this.robot_diameter);
            numY = floor(arena_height / this.robot_diameter);
            values = randperm(numX * numY, this.number_of_agents);
            
            initial_poses = zeros(3, this.number_of_agents);
            
            for i = 1:this.number_of_agents
               [x, y] = ind2sub([numX numY], values(i));
               x = x*this.robot_diameter - (arena_width/2); 
               y = y*this.robot_diameter - (arena_height/2);
               initial_poses(1:2, i) = [x ; y];
            end
                                                        
            initial_poses(3, :) = rand(1, this.number_of_agents)*2*pi;
            robotarium_obj = Robotarium(this.number_of_agents, this.save_data, initial_poses);            
        end
    end  
end

