classdef APIBuilder < handle
    %APIBUILDER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        number_of_agents
        save_data = true
    end
    
    methods (Abstract)
        % Builds the Robotarium object.  Definitely backend/sim dependent.
        get_available_agents(this);
        build(this);
    end
    
    methods                         
        function this = set_number_of_agents(this, number_of_agents)
           this.number_of_agents = number_of_agents;
        end
        
        function this = set_save_data(this, save_data)
           this.save_data = save_data; 
        end
    end   
end

