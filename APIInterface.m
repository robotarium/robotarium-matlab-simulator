classdef APIInterface < handle
    %APIINTERFACE This is an interface for the Robotarium class that
    %ensures the simulator and the robots match up properly.  You should
    %definitely NOT MODIFY this file.  Also, don't submit this file with
    %your algorithm. 
    
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

