classdef Robotarium < APIInterface
    %ROBOTARIUM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant) 
       lcmParser = parsers.Parsers.parser;
    end
    
    properties (GetAccess = public, SetAccess = private)
        
        %Data logging parameters
        states
        tempStates
        filePath
        saveLength
        saveEvery
        prevIters
        iters
        timeStep
        timeStamp
        
        %Dynamics
        numAgents
        dx
        
        %Controllers (functions)
        positionController
        
        %Visualization
        robotHandle
        boundaries
        robotBody
        
    end
    
    methods    
        
        % CHECK THESE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function this = Robotarium()

            msg = robotariumMessages.apiReceiveDataMsg_t(this.lcmParser.getMessageBlocking('robotStates').message.data);
            this.states = msg.data;
            this.timeStamp = msg.timestamp;
        end
        
        function this = initialize(this, N) 
            %ROBOTARIUM Constructor for the robotarium 
            % N - number of agents to simulate

            %Approx. time update for the Robotarium           
            this.numAgents = N;            
            this.dx = zeros(2, N);

            %Controllers (CLF default)
            this.positionController = @positionCLF; 

            %Save data parameters

            this.boundaries = [-0.6, 0.6, -0.35, 0.35];                  
            this.saveLength = 0;           
            this.saveEvery = 0;
            this.iters = 1;            
            this.prevIters = 1;            
            this.tempStates = [];   

            this.numAgents = N;
        end

        function this = setPositionController(this, controller)
            %SETPOSITIONCONTROLLER Sets the position controller to use for
            %the simulation 
            % controller (function): position (go-to-goal) controller 
            this.positionController = controller;
        end

        function setVelocities(this, ids, vs)  
            %SETVELOCITIES Sets the velocities of the current agents
            % obj = SETVELOCITIES(identities, velocities)
            % identities: Identities of agents whose velocities to set
            % velocities: Velocities to set

            % Threshold velocities 

            [~, N] = size(vs);
            for i = 1:N
                if(abs(vs(1, i)) > 0.1) 
                   vs(1, i) = 0.1*sign(vs(1,i)); 
                end

                if(abs(vs(2, i)) > (2*pi))
                   vs(2, i) = 2*pi*sign(vs(2, i)); 
                end
            end

            this.dx(:, ids) = vs;
            this.states(5:6, ids) = vs;
        end

        %Functions for setting positions (go-to-goal behavior)

        function this = setPositions(this, ids, ps) 
            %SETPOSITIONS Sets velocities for the agents via the position
            %controller
            % obj = SETPOSITIONS(obj, identities, positions) 
            % identities: identities of agent positions to set
            % positions: Goal positions of agents 2 x N

            this.setVelocities(ids, this.positionController(this.states(2:4, ids), ps));
        end

        function neighbors = getDDiskNeighbors(this, id, r) 
            %GETDDISKNEIGHBORS Gets the neighbors of a particular agent
            %within r distance in the 2 norm
            % neighbors = GETDDISKNEIGHBORS(obj, identities, radius) 
            % identity: identy of agent whose neighbors to get 
            % radius: radius of delta disk

            neighbors = zeros(1, this.numAgents);
            count = 0;

            for i = 1:this.numAgents
                if((i ~= id) && (norm(this.states(2:3, id) - this.states(2:3, i)) < r))
                    count = count + 1;
                    neighbors(:, count) = i;
                end
            end

            if(count == 0) 
                neighbors = [];
            else 
               neighbors = neighbors(1:count); 
            end
        end

        %Gets topological neighbors given a graph laplacian, L
        function neighbors = getTopNeighbors(this, id, L)

            neighbors = zeros(1, this.numAgents);
            count = 0;

            for i = 1:this.numAgents
                if((i ~= id) && (L(id, i) ~= 0))
                    count = count + 1;
                    neighbors(:, count) = i;
                end
            end

            if(count == 0) 
                neighbors = [];
            else
                neighbors = neighbors(1:count); 
            end
        end

        %Gets the (x, y, theta) poses of the robots
        function poses = getPoses(this)
            %GETPOSES Get the current poses of the agents.  Will wait until
            %new information is available
            % Message packet structure
            % 1. IP
            % 2. x-coordinate
            % 3. y-coordinate 
            % 4. theta
            % 5. linear velocity v
            % 6. angular velocity w
            % 7. battery voltage V
            % 8. current draw I
            % 9. distance to target 

            msg = robotariumMessages.apiReceiveDataMsg_t(this.lcmParser.getMessageBlocking('robotStates').message.data);
            this.timeStamp = msg.timestamp;
            msg = msg.data;
            this.states = msg(:, 1:this.numAgents);
            poses = this.states(2:4, :);
        end
        
        function numAgents = getAvailableAgents(this) 
            [~, numAgents] = size(this.states);
        end

        function step(this)         

            %Construct message
            msg = robotariumMessages.apiRequestDataMsg_t();
            msg.IP          = 0;
            msg.msgType     = double(uint32(MESSAGE_TYPES.SET_ALL_VELOCITIES));
            msg.parameters  = [this.states(1, :) ; this.dx];
            [msg.nRow, msg.nCol] = size(msg.parameters);

            %Send message
            this.lcmParser.sendMessage('apiRequestData', msg);
            this.save();
        end

        function iters = time2iters(this, time) 
            iters = ceil(time / this.timeStep);
        end

        function this = setSaveParameters(this, filePath, length, every) 
            %SETSAVEPARAMETERS Sets the state saving parameters for the
            %simulation           
            % filePath: path to save file 
            % length: number of state iterations to save 
            % every: save per 'every' iterations

            this.saveLength = length;
            this.saveEvery = every;
            this.tempStates = zeros(5 * this.numAgents, every);    
            this.filePath = filePath;
            robotStates = zeros(5 * this.numAgents + 1, length);

            save(strcat(filePath, '.mat'), 'robotStates', '-v7.3');
        end  
        
        function save(this)           
            if(~isempty(this.tempStates))
                if(this.prevIters < this.saveLength)
                    m = matfile(strcat(this.filePath, '.mat'), 'Writable', true);                    

                    if((this.iters - 1) == this.saveEvery)

                        m.robotStates(:, this.prevIters:(this.prevIters + this.iters - 2)) = this.tempStates;

                        % Save previous iteration count
                        this.prevIters = this.prevIters + this.saveEvery;

                        % Set current iterations to 1
                        this.iters = 1;            
                    end

                    this.tempStates(1:(5*this.numAgents), this.iters) = reshape(this.states(2:6, :), [], 1);
                    this.tempStates(5*this.numAgents + 1, this.iters) = this.timeStamp;
                    this.iters = this.iters + 1;
                else
                    display('Requested data saved.')
                    this.tempStates = [];
                end
            end
        end
    
        function shutdown(this) 
            this.lcmParser.shutdown(); 
        end                
        
    end
    
end


