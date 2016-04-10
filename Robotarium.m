classdef Robotarium < APIInterface
    %ROBOTARIUM Simulator interface for the Robotarium
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = private)
        timeStep
    end
    
    properties(GetAccess = private, SetAccess = private)
        
        %Data logging parameters
        states
        tempStates
        saveLength
        saveEvery
        prevIters
        iters
        filePath
        
        %Dynamics
        numAgents
        linearVelocityCoef 
        angularVelocityCoef
        
        %Controllers (functions)
        positionController
        
        %Visualization
        robotHandle
        boundaries
        robotBody
        
        %Barrier Certificates 
%         gamma = 1e4 
%         safetyRadius = 0.1
%         maxLinearVelocity = 0.1
%         maxAngularVelocity = 2*pi
%         diffeomorphismGain = 0.05
    end
    
    methods
        %Functions for setting velocities
        
        function this = Robotarium()    
            %Random, typical value
            this.numAgents = 4;
        end
        
        function this = initialize(this, N)
           %INITIALIZE Initializer for the robotarium 
            % N - number of agents to simulate
            
            %Coefficients determined from system identification
            this.linearVelocityCoef = 0.7;
            this.angularVelocityCoef = 0.7;
            
            %Generate random states...sloppy
            this.states = zeros(5, N);
            
            %this.states(1:3, :) = unifrnd(0, 0.6, 3, N) - 0.3;
            
            numX = floor(1.2 / this.safetyRadius);
            numY = floor(0.7 / this.safetyRadius);
            values = randperm(numX * numY, N);
            
            for i = 1:N
               [x, y] = ind2sub([numX numY], values(i));
               x = x*this.safetyRadius - 0.6; 
               y = y*this.safetyRadius - 0.35;
               this.states(1:2, i) = [x ; y];
            end
        
                                                
            this.states(3, :) = rand(1, N)*2*pi;
            
            %Approx. time update for the Robotarium
            this.timeStep = 0.033;            
            this.numAgents = N;            
            
            %Controllers (CLF default)
            this.positionController = @positionCLF; 
            
            %Save data parameters
            
            this.boundaries = [-0.6, 0.6, -0.35, 0.35];         
            this.InitRobotVisualize();          
            this.saveLength = 0;           
            this.saveEvery = 0;
            this.iters = 1;            
            this.prevIters = 1;            
            this.tempStates = [];  
        end
                       
        function this = setPositionController(this, controller)
            %SETPOSITIONCONTROLLER Sets the position controller to use for
            %the simulation 
            % controller (function): position (go-to-goal) controller 
            this.positionController = controller;
        end
        
        function this = setVelocities(this, ids, vs)  
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
            
            this.states(4:5, ids) = vs;
        end
        
        %Functions for setting positions (go-to-goal behavior)
        
        function setPositions(this, ids, ps) 
            %SETPOSITIONS Sets velocities for the agents via the position
            %controller
            % obj = SETPOSITIONS(obj, identities, positions) 
            % identities: identities of agent positions to set
            % positions: Goal positions of agents 2 x N
            
            this.setVelocities(ids, this.positionController(this.states(:, ids), ps));
        end
        
        function neighbors = getDDiskNeighbors(this, id, r) 
            %GETDDISKNEIGHBORS Gets the neighbors of a particular agent
            %within r distance in the 2 norm
            % neighbors = GETDDISKNEIGHBORS(obj, identity, radius) 
            % identity: identy of agent whose neighbors to get 
            % radius: radius of delta disk
            
            neighbors = zeros(1, this.numAgents);
            count = 0;
            
            for i = 1:this.numAgents
                if((i ~= id) && (norm(this.states(1:2, id) - this.states(1:2, i)) < r))
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
            %GETTOPNEIGHBORS Gets the topological neighbors of an agent,
            %given a graph Laplacian
            % neighbors = GETTOPNEIGHBORS(obj, identity, laplacian)
            % identity: identity of agents whose neighbors to get
            % laplacian: Graph Laplacian of the communication topology
            
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
            poses = this.states;
        end
                      
        function step(this)         
                      
            %Vectorize update to states
            i = 1:this.numAgents;
            
            %Update velocities using unicycle dynamics 
            this.states(1, i) = this.states(1, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*cos(this.states(3, i));
            this.states(2, i) = this.states(2, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*sin(this.states(3, i));
            this.states(3, i) = this.states(3, i) + this.angularVelocityCoef*this.timeStep.*this.states(5, i);            
            
            %Ensure that we're in the right range
            %this.states(3, i) = atan2(sin(this.states(3, i)) , cos(this.states(3, i)));
            
            this.save();
            this.drawRobots();
            pause(this.timeStep); 
        end
        
        function numAgents = getAvailableAgents(this) 
            numAgents = this.numAgents;
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
            
            robotStates = zeros(5 * this.numAgents, length);
            
            save(filePath, 'robotStates', '-v7.3');
        end
    end
    
    %PRIVATE METHODS
    methods(Access = private)   
                                    
        function save(this)           
            if(~isempty(this.tempStates))
                if(this.prevIters <= this.saveLength)
                    m = matfile(strcat(this.filePath, '.mat'), 'Writable', true);                    

                    if((this.iters - 1) == this.saveEvery)

                        m.robotStates(:, this.prevIters:(this.prevIters + this.iters - 2)) = this.tempStates;

                        % Save previous iteration count
                        this.prevIters = this.prevIters + this.saveEvery;

                        % Set current iterations to 1
                        this.iters = 1;            
                    end

                    this.tempStates(:, this.iters) = reshape(this.states, [], 1);
                    this.iters = this.iters + 1;
                else
                    display('Requested data saved.')
                    this.tempStates = [];
                end
            end
        end
            
        function drawRobots(r)
            for ii = 1:r.numAgents
                x  = r.states(1, ii);
                y  = r.states(2, ii);
                th = r.states(3, ii);
                poseTransformationMatrix = [...
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0                    0                   1   ];
                robotBodyTransformed = r.robotBody*poseTransformationMatrix';
                set(r.robotHandle{ii},'Vertices', robotBodyTransformed);
            end
        end
        
        function InitRobotVisualize(r)
            % Initialize variables
            robotDiameter = 0.03;
            numRobots = r.numAgents;
            
            % Scale factor (max. value of single Gaussian)
            scaleFactor = 0.5;  
            figPhi = figure(1);
            set(figPhi,'color','white','menubar','none');
            
            % Set axis
            robotPlaneAxes = gca;
            
            % Limit view to xMin/xMax/yMin/yMax
            axis(robotPlaneAxes,[r.boundaries(1),r.boundaries(2),r.boundaries(3),r.boundaries(4)])
            caxis([0,1.5*scaleFactor])
            set(robotPlaneAxes,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1])
            
            % Store axes
            axis(robotPlaneAxes,'off')
            
            set(robotPlaneAxes,'position',[0 0 1 1],'units','normalized','YDir','normal')

            hold on % "This ride's about to get bumpy!"

            % Let's jump through hoops to make the robot diameter look to data scale
            curUnits = get(robotPlaneAxes, 'Units');
            set(robotPlaneAxes, 'Units', 'Points');
            set(robotPlaneAxes, 'Units', curUnits);

            % Define custom patch variables
            gritsBotBase =  [ ...
            -sqrt(2)/2 -sqrt(2)/2  1;
             sqrt(2)/2 -sqrt(2)/2  1;
             sqrt(2)/2  sqrt(2)/2  1;
            -sqrt(2)/2  sqrt(2)/2  1];
            gritsBotBase(:,1:2) = gritsBotBase(:,1:2)*robotDiameter;
            gritsBotWheel =  [ ...
            -sqrt(2)/2 -sqrt(2)/2  1;
             sqrt(2)/2 -sqrt(2)/2  1;
             sqrt(2)/2  sqrt(2)/2  1;
            -sqrt(2)/2  sqrt(2)/2  1];
            gritsBotWheel(:,1:2) = gritsBotWheel(:,1:2)...
            *diag([ robotDiameter/3 robotDiameter/6]);
            gritsBotLeftWheel =  bsxfun(@plus,gritsBotWheel,[ 0  7/6*sqrt(2)/2*robotDiameter 0 ]);
            gritsBotRightWheel = bsxfun(@plus,gritsBotWheel,[ 0 -7/6*sqrt(2)/2*robotDiameter 0 ]);
            gritsBotTailPinAngle = (pi*8/9:pi/18:pi*10/9)';
            gritsBotTailPin = [ ...
            -sqrt(2)/2 sin(gritsBotTailPinAngle(1)) 1;
            cos(gritsBotTailPinAngle) sin(gritsBotTailPinAngle) ones(size(gritsBotTailPinAngle));
            0.95*cos(gritsBotTailPinAngle(end:-1:1)) 0.95*sin(gritsBotTailPinAngle(end:-1:1)) ones(size(gritsBotTailPinAngle));
            -sqrt(2)/2 0.95*sin(gritsBotTailPinAngle(1)) 1];
            gritsBotTailPin(:,1:2) = gritsBotTailPin(:,1:2)*robotDiameter;
            gritsBotBaseColor = [ 0.5843    0.1294    0.9647 ];
            gritsBotWheelColor = [ 0.0 0.0 0.0 ];
            gritsBotTailPinColor = [ 0.8 0.8 0.8];
            gritsBotTagWhite = [ 1 1 1 ];
            gritsBotTagBlack = [ 0 0 0 ];
            
            % Define an unit circle circumscribed rectangle
            rectangleBox = [ ...
            -sqrt(2)/2 -sqrt(2)/2;
             sqrt(2)/2 -sqrt(2)/2;
             sqrt(2)/2  sqrt(2)/2;
            -sqrt(2)/2  sqrt(2)/2];
            arucoTagScale = 0.8;
            rectangleBox = rectangleBox*arucoTagScale*robotDiameter;
            arucoBoxScale = 0.15;
            arucoBoxShiftScale = 1*arucoBoxScale;
            gritsBotTag =  [ ...
            % Outer white border
            [rectangleBox ones(4,1)];
            % Inner black border
            [0.9*rectangleBox ones(4,1)];
            % Begin 4x4 aruco tag grid
            % First Row
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,1
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,2
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,3
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,4
            % Second Row
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,1
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,2
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,3
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,4
            % Third Row
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,1
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,2
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,3
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,4
            % Fourth Row
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1) 3*rectangleBox(3,2)]) ones(4,1)]; % Grid: 4,1
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1) 3*rectangleBox(3,2)]) ones(4,1)]; % Grid: 4,2
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1) 3*rectangleBox(3,2)]) ones(4,1)]; % Grid: 4,3
            [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1) 3*rectangleBox(3,2)]) ones(4,1)]];% Grid: 4,4

            % Define common patch variables
            r.robotBody = [gritsBotLeftWheel;gritsBotRightWheel;gritsBotTailPin;gritsBotBase;gritsBotTag];

            robotFaces =  [...
            1:size(gritsBotWheel,1)  NaN(1,8-size(gritsBotBase,1)+size(gritsBotWheel,1));
            1+size(gritsBotWheel,1):2*size(gritsBotWheel,1) NaN(1,4-size(gritsBotBase,1)+2*size(gritsBotWheel,1));
            1+2*size(gritsBotWheel,1):2*size(gritsBotWheel,1)+size(gritsBotTailPin,1);
            1+2*size(gritsBotWheel,1)+size(gritsBotTailPin,1):2*size(gritsBotWheel,1)+size(gritsBotTailPin,1)+size(gritsBotBase,1) NaN(1,12-size(gritsBotBase,1));
            %
            [reshape(bsxfun(@plus,(1:18*size(rectangleBox,1)),size(gritsBotBase,1)+2*size(gritsBotWheel,1)+size(gritsBotTailPin,1)),[],18)' NaN(18,8)]];

            r.robotHandle = cell(numRobots);
            for ii = 1:numRobots
                x  = r.states(1, ii);
                y  = r.states(2, ii);
                th = r.states(3, ii);
                poseTransformationMatrix = [...
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0                    0                   1   ];
                robotBodyTransformed = r.robotBody*poseTransformationMatrix';
                robotColor = [gritsBotWheelColor;gritsBotWheelColor;gritsBotTailPinColor;gritsBotBaseColor;
                    gritsBotTagWhite;gritsBotTagBlack;
                    bsxfun(@times,ones(16,3),(rand(16,1)>0.5))];
                r.robotHandle{ii} = patch(...
                          'Vertices', robotBodyTransformed, ...
                          'Faces',robotFaces, ...
                          'FaceColor', 'flat', ...
                          'FaceVertexCData',robotColor, ...
                          'EdgeColor','none');
            end
        end    
    end
    
end

