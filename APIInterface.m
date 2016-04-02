classdef APIInterface < handle
    %APIINTERFACE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Abstract)
        
        %Initializer 
        initialize(this, N);
        
        %Setters 
        setPositionController(this, controller);
        setVelocities(this, ids, vs)        
        setPositions(this, ids, ps)
          
        %Getters 
        getDDiskNeighbors(this, id, r) 
        getTopNeighbors(this, id, L)
        getPoses(this)
        getAvailableAgents(this);
    
        
        %Update functions
        step(this);
        time2iters(this, time);
                        
        %Save parameters
        setSaveParameters(this, filePath, length, chunks);
        
    end
    
end

