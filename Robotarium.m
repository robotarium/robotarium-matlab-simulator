classdef Robotarium < APIInterface
    %ROBOTARIUM The Robotarium class inherits from the API Interface, which
    % connects this simulator interface with the actual, physical robots on
    % the Robotarium.  Thus, please: 
    % DO NOT MODIFY THIS FILE.  IF YOU DO, YOUR CODE WILL NOT RUN.
    
    % Allow users to have access to the figure handle for the Robotarium 
    % as well as the current time step; but they can't modify the value.
    properties(GetAccess = public, SetAccess = private)
        
        % Time step for the Robotarium
        timeStep
        
        % Figure handle for simulator
        figureHandle
        
        % Arena parameters
        boundaries = [-0.6, 0.6, -0.35, 0.35];     
    end
    
    % Make sure that the user doesn't have access to variables that are not
    % available on the actual hardware!
    properties(GetAccess = private, SetAccess = private)
        
        % A flag to ensure that the users do not call get_poses() more than
        % once per iteration of their algorithm.
        checked_poses_already = false
        
        % Parameters for data logging to *.mat files.
        states
        tempStates
        saveLength
        saveEvery
        prevIters
        iters
        filePath
        previousTimestep
        
        % System ID parameters for unicycle model
        numAgents
        xLinearVelocityCoef = 0.86
        yLinearVelocityCoef = 0.81
        angularVelocityCoef = 0.46
        maxLinearVelocity = 0.1
        maxAngularVelocity = 2*pi
        robotDiameter = 0.08
        
        %  Higher-level controllers for position (rather than velocity)
        positionController
        
        % Visualization parameters for displaying the simulator
        robotHandle
        boundaryPoints
        robotBody
        offset = 0.05
    end
    
    methods
        %Functions for setting velocities
        
        function this = Robotarium()    
            %ROBOTARIUM This function returns a Robotarium class object for
            %use in the simulator.  
            % In particular, this objects allows you to interact with the
            % simulated agents, controlling their velocities and iterating
            % through the simulation.  Before use, you must call
            % INITIALIZE() with the number of agents that you wish to use.
            this.numAgents = 4;
        end
        
        function this = initialize(this, N)
           %INITIALIZE Initializes the Robotarium object with the desired number of agents.
           % obj = INITIALIZE(obj, number_of_agents)
           % N: The number of agents to use in the simulation.  This
           % function must be called before beginning a simulation of your
           % algorithm.
            
            this.states = zeros(5, N);
                     
            % Make sure that we place robots in disparate locations to
            % prevent collisions on initial conditions
            numX = floor(1.2 / this.robotDiameter);
            numY = floor(0.7 / this.robotDiameter);
            values = randperm(numX * numY, N);
            
            for i = 1:N
               [x, y] = ind2sub([numX numY], values(i));
               x = x*this.robotDiameter - 0.6; 
               y = y*this.robotDiameter - 0.35;
               this.states(1:2, i) = [x ; y];
            end
                                                        
            this.states(3, :) = rand(1, N)*2*pi;
            
            %Approx. time update for the Robotarium
            this.timeStep = 0.033;            
            this.numAgents = N;            
            
            %Controllers (CLF default)
            this.positionController = @positionCLF; 
            
            %Save data parameters 
                               
            this.boundaryPoints = {[-0.6, 0.6, 0.6, -0.6], [-0.35, -0.35, 0.35, 0.35]}; 
                                   
            this.InitRobotVisualize();          
            this.saveLength = 0;           
            this.saveEvery = 0;
            this.iters = 1;            
            this.prevIters = 1;            
            this.tempStates = [];  
            this.previousTimestep = tic;
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
            % ids: Identities of agents whose velocities to set
            % vs: Velocities to set           

            N = size(vs, 2);
                     
            % Threshold velocities
            for i = 1:N
                if(abs(vs(1, i)) > this.maxLinearVelocity) 
                   vs(1, i) = this.maxLinearVelocity*sign(vs(1,i)); 
                end
                
                if(abs(vs(2, i)) > this.maxAngularVelocity)
                   vs(2, i) = this.maxAngularVelocity*sign(vs(2, i)); 
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
            
           N = size(ps, 2); 
           
           if(N ~= this.numAgents)
              warning('The second dimension length of supplied positions (%i) was not equal to the number of specified agents (%i)', N, this.numAgents);
           end
            
            this.setVelocities(ids, this.positionController(this.states(:, ids), ps));
        end
        
        function neighbors = getDDiskNeighbors(this, id, r) 
            %GETDDISKNEIGHBORS Returns identities of neighbors within
            %euclidean distance r
            % neighbors = GETDDISKNEIGHBORS(obj, identity, radius) 
            % identity: identy of agent
            % radius: radius of disk;
            
            
            assert(id <= this.numAgents, 'Error: supplied id (%i) was greater than the number of agents (%i)', id, this.numAgents);                       
            
            % Make sure we don't count ourselves
            ids = 1:this.numAgents; 
            ids(id) = [];
            
            distances = arrayfun(@(x) norm(this.states(1:2, id) - this.states(1:2, x)), ids);           
            neighbors = ids(find(distances <= r));                                     
        end
        
        function neighbors = getTopNeighbors(this, id, L)
            %GETTOPNEIGHBORS Gets the topological neighbors of an agent,
            %given a graph Laplacian
            % neighbors = GETTOPNEIGHBORS(obj, identity, laplacian)
            % identity: identity of agents whose neighbors to get
            % laplacian: Graph Laplacian of the communication topology
            
            assert(id <= this.numAgents, 'Error: supplied id (%i) was greater than the number of agents (%i)', id, this.numAgents)
            
            L(id, id) = 0;
            neighbors = find(L(id, :) ~= 0);            
        end
        
        %Gets the (x, y, theta) poses of the robots
        function poses = getPoses(this)
            
            assert(~this.checked_poses_already, 'Can only check get poses once per call of step()!');
            
            poses = this.states(1:3, :);
            
            %Include delay to mimic behavior of real system
            this.previousTimestep = tic;
            
            %Make sure it's only called once per iteration
            this.checked_poses_already = true;
        end
                      
        function step(this)         
                      
            %Vectorize update to states
            i = 1:this.numAgents;
            
            total_time = this.timeStep + max(0, toc(this.previousTimestep) - this.timeStep);
            
            %Update velocities using unicycle dynamics 
            this.states(1, i) = this.states(1, i) + this.xLinearVelocityCoef*total_time.*this.states(4, i).*cos(this.states(3, i));
            this.states(2, i) = this.states(2, i) + this.yLinearVelocityCoef*total_time.*this.states(4, i).*sin(this.states(3, i));
            this.states(3, i) = this.states(3, i) + this.angularVelocityCoef*total_time.*this.states(5, i);            
            
            %Ensure that we're in the right range
            this.states(3, i) = atan2(sin(this.states(3, i)), cos(this.states(3, i)));
            
            %Allow getting of poses again 
            this.checked_poses_already = false;                                    
            
            this.save();
            this.drawRobots();
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
            
            save(filePath, 'robotStates', '-v7.3')
        end
    end
    
    %PRIVATE METHODS
    methods(Access = private)   
                                    
        function save(this)           
            if(~isempty(this.tempStates))
                if(this.prevIters <= this.saveLength)
                    m = matfile(this.filePath, 'Writable', true);                    

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
            gcf_ = gcf;
            previousFigureNumber = gcf_.Number;            
            figure(r.figureHandle.Number)
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
            drawnow 
            figure(previousFigureNumber)
        end
        
        function InitRobotVisualize(r)
            % Initialize variables
            robotDiameter = 0.03;
            numRobots = r.numAgents;
            
            % Scale factor (max. value of single Gaussian)
            scaleFactor = 0.5;  
            figPhi = figure;
            r.figureHandle = figPhi;
            
            % Plot Robotarium boundaries
            patch('XData', r.boundaryPoints{1}, 'YData', r.boundaryPoints{2}, ...
            'FaceColor', 'none', ...
            'LineWidth', 3, ... 
            'EdgeColor', [0, 0.74, 0.95]);
            
            %plot(im)
            set(figPhi,'color','white','menubar','none');
            
            % Set axis
            robotPlaneAxes = gca;
            
            % Limit view to xMin/xMax/yMin/yMax
            axis(robotPlaneAxes,[r.boundaries(1) - r.offset,r.boundaries(2)+r.offset,r.boundaries(3)-r.offset,r.boundaries(4)+r.offset])
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

