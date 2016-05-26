classdef Robotarium < APIInterface
    %ROBOTARIUM Simulator interface for the Robotarium
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = private)
        timeStep
        figureHandle
        S = 0
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
        linearVelocityCoef = 0.7
        angularVelocityCoef = 0.7
        maxLinearVelocity = 0.1
        maxAngularVelocity = 2*pi
        
        %Controllers (functions)
        positionController
        
        %Visualization
        robotHandle
        boundaries
        boundaryPoints
        robotBody
        offset = 0.05
        
        %Barrier Certificates 
        gamma = 1e4 
        safetyRadius = 0.1
        diffeomorphismGain = 0.05
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
            
            this.states = zeros(5, N);
                     
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
                               
            this.boundaryPoints = {[-0.6, 0.6, 0.6, -0.6], [-0.35, -0.35, 0.35, 0.35]}; 
                                   
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
            poses = this.states(1:3, :);
        end
        
        
        function step(this) ;
            if 1
                b=[-0.6000    0.6000   -0.3500    0.3500]./1.05;
                robotDiameter = 0.03;
                wheelSlipRobots=0.05; % 0 for no slip, 1 for full slip
                wheelSlipWall=0.1;
                slide=0.1;
                bounce1=0.2;
                bounce2=0.2;
                for i = 1:this.numAgents;
                    Collision=0;
                    % Find candidate new state for agent i
                    cx=this.states; % Where the robot has been commanded to go
                    cdx=this.states(4:5,:); % How the robot has been commanded to move
                    cx(1, i) = this.states(1, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*cos(this.states(3, i)); %
                    cx(2, i) = this.states(2, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*sin(this.states(3, i));
                    cx(3, i) = this.states(3, i) + this.angularVelocityCoef*this.timeStep.*this.states(5, i);
                    th=cx(3,i);
                    Pt=[[1,1,1,1].*cx(1,i);[1,1,1,1].*cx(2,i)]+[cos(th), -sin(th);sin(th),cos(th)]*[0.72,-0.72,-0.72,0.72;.81,.81,-.81,-.81].*robotDiameter;  % Defining Points of Body i
                    xHati=(Pt(:,4)-Pt(:,3))./(1.62*robotDiameter); % Unit vector going forward in the body frame
                    
                    % Velocity only updated if no collisions are caused
                    for j=1:this.numAgents; % Overlap Test
                        if  i~=j;
                            thj=cx(3,j);
                            Qt=[[1,1,1,1].*cx(1,j);[1,1,1,1].*cx(2,j)]+[cos(thj), -sin(thj);sin(thj),cos(thj)]*[0.72,-0.72,-0.72,0.72;.81,.81,-.81,-.81].*robotDiameter; % Defining points of body j
                            rij=(cx(1:2,i)-cx(1:2,j));
                            if  norm(rij)<2*robotDiameter; % PHASE I
                                aj=Qt(:,1); bj=Qt(:,2);cj=Qt(:,3);dj=Qt(:,4);Aj=Qt(:,2)-Qt(:,1);Bj=Qt(:,3)-Qt(:,2);Cj=Qt(:,4)-Qt(:,3);Dj=Qt(:,1)-Qt(:,4);
                                nAj=[0,1;-1,0]*Aj;nBj=[0,1;-1,0]*Bj; nCj=[0,1;-1,0]*Cj;nDj=[0,1;-1,0]*Dj;nj=[nAj,nBj,nCj,nDj];
                                Aj_mag=norm(Aj);Bj_mag=norm(Bj);Cj_mag=norm(Cj);Dj_mag=norm(Dj);
                                ai=Pt(:,1);bi=Pt(:,2);ci=Pt(:,3);di=Pt(:,4);Ai=Pt(:,2)-Pt(:,1);Bi=Pt(:,3)-Pt(:,2);Ci=Pt(:,4)-Pt(:,3);Di=Pt(:,1)-Pt(:,4);
                                nAi=[0,1;-1,0]*Ai;nBi=[0,1;-1,0]*Bi; nCi=[0,1;-1,0]*Ci;nDi=[0,1;-1,0]*Di;ni=[nAj,nBj,nCj,nDj];
                                Ai_mag=norm(Aj); Bi_mag=norm(Bj);Ci_mag=norm(Cj);Di_mag=norm(Dj);ni=[nAj,nBj,nCj,nDj];
                                % Test if any point in i is in j
                                for k=1:4; % PHASE2 has i entered j
                                    distVec1=[((Aj(1)*(Pt(2,k)-aj(2)))-Aj(2)*(Pt(1,k)-(aj(1))))/Aj_mag, ((Bj(1)*(Pt(2,k)-bj(2)))-Bj(2)*(Pt(1,k)-(bj(1))))/Bj_mag,...
                                        ((Cj(1)*(Pt(2,k)-cj(2)))-Cj(2)*(Pt(1,k)-(cj(1))))/Cj_mag, ((Dj(1)*(Pt(2,k)-dj(2)))-Dj(2)*(Pt(1,k)-(dj(1))))/Dj_mag];
                                    if distVec1(1)>=0 && distVec1(2)>=0 && distVec1(3)>=0 && distVec1(4)>=0; % Collision Case 1 (see notes)
                                        Collision=1;
                                        [minnum,minInd]=min(distVec1(:));
                                        nhatcol=([nj(1,minInd),nj(2,minInd)]'./norm([nj(1,minInd),nj(2,minInd)]));
                                        WallVec=[0,-1;1,0]*nhatcol; %vector // to wall
                                        WallVecHat=WallVec/norm(WallVec);
                                        newVel=dot(cdx(1,i).*xHati,WallVecHat).*WallVecHat*slide+(norm(cdx(1,i)).*nhatcol)*bounce1./2+(rij./norm(rij))*bounce2./3;
                                        
                                        cx(1, i) = this.states(1, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*newVel(1);
                                        cx(2, i) = this.states(2, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*newVel(2);
                                        cx(3, i) = this.states(3, i) + (rand-0.5)./80;
                                        
                                        th=cx(3,i);
                                        Pt=[[1,1,1,1].*cx(1,i);[1,1,1,1].*cx(2,i)]+[cos(th), -sin(th);sin(th),cos(th)]*[0.8,-0.8,-0.8,0.8;.9,.9,-.9,-.9].* 0.9*robotDiameter ; % Defining Points of Body i
                                        
                                        penalty=((norm(dot(cdx(1,i).*xHati,-nhatcol)))^2)./this.numAgents;
                                        this.S=this.S+penalty;
                                        
                                        % Agent i now has a new position, we need to run the same tests over again to check if the NEW position is going to cause a collision
                                        % If it does then don't update
                                        for j2=1:this.numAgents; % Overlap Test: has i entered (some other) j
                                            if  i~=j2;
                                                thj2=cx(3,j2);
                                                Qt=[[1,1,1,1].*cx(1,j2);[1,1,1,1].*cx(2,j2)]+[cos(thj2), -sin(thj2);sin(thj2),cos(thj2)]*[0.8,-0.8,-0.8,0.8;.9,.9,-.9,-.9].* 0.9*robotDiameter; % Defining points of body j
                                                rij=(cx(1:2,i)-cx(1:2,j2));
                                                if norm(cx(1:2,i)-cx(1:2,j2))<2.0*robotDiameter; % PHASE I
                                                    aj=Qt(:,1); bj=Qt(:,2);cj=Qt(:,3);dj=Qt(:,4);Aj=Qt(:,2)-Qt(:,1);Bj=Qt(:,3)-Qt(:,2);Cj=Qt(:,4)-Qt(:,3);Dj=Qt(:,1)-Qt(:,4);
                                                    nAj=[0,1;-1,0]*Aj;nBj=[0,1;-1,0]*Bj; nCj=[0,1;-1,0]*Cj;nDj=[0,1;-1,0]*Dj;nj=[nAj,nBj,nCj,nDj];
                                                    Aj_mag=norm(Aj);Bj_mag=norm(Bj);Cj_mag=norm(Cj);Dj_mag=norm(Dj);
                                                    ai=Pt(:,1);bi=Pt(:,2);ci=Pt(:,3);di=Pt(:,4);Ai=Pt(:,2)-Pt(:,1);Bi=Pt(:,3)-Pt(:,2);Ci=Pt(:,4)-Pt(:,3);Di=Pt(:,1)-Pt(:,4);
                                                    nAi=[0,1;-1,0]*Ai;nBi=[0,1;-1,0]*Bi; nCi=[0,1;-1,0]*Ci;nDi=[0,1;-1,0]*Di;ni=[nAj,nBj,nCj,nDj];
                                                    Ai_mag=norm(Aj); Bi_mag=norm(Bj);Ci_mag=norm(Cj);Di_mag=norm(Dj);ni=[nAj,nBj,nCj,nDj];
                                                    
                                                    % Test if any point in i is in j or if any j has entered i
                                                    for k=1:4; % PHASE2 has i entered j
                                                        distVec1=[((Aj(1)*(Pt(2,k)-aj(2)))-Aj(2)*(Pt(1,k)-(aj(1))))/Aj_mag, ((Bj(1)*(Pt(2,k)-bj(2)))-Bj(2)*(Pt(1,k)-(bj(1))))/Bj_mag,...
                                                            ((Cj(1)*(Pt(2,k)-cj(2)))-Cj(2)*(Pt(1,k)-(cj(1))))/Cj_mag, ((Dj(1)*(Pt(2,k)-dj(2)))-Dj(2)*(Pt(1,k)-(dj(1))))/Dj_mag];
                                                        distVec2=[((Ai(1)*(Qt(2,k)-ai(2)))-Ai(2)*(Qt(1,k)-(ai(1))))/Ai_mag, ((Bi(1)*(Qt(2,k)-bi(2)))-Bi(2)*(Qt(1,k)-(bi(1))))/Bi_mag,...
                                                            ((Ci(1)*(Qt(2,k)-ci(2)))-Ci(2)*(Qt(1,k)-(ci(1))))/Ci_mag, ((Di(1)*(Qt(2,k)-di(2)))-Di(2)*(Qt(1,k)-(di(1))))/Di_mag];
                                                        
                                                        if distVec1(1)>=0 && distVec1(2)>=0 && distVec1(3)>=0 && distVec1(4)>=0; % Collision Case 1 (see notes)
                                                            Collision=3;
                                                        elseif distVec2(1)>=0 && distVec2(2)>=0 && distVec2(3)>=0 && distVec2(4)>=0; % Collision Case 2 (see notes)
                                                            Collision=4;
                                                        end
                                                    end
                                                end
                                            end
                                        end % End if j loop: every other robot tested with i
                                        
                                    else
                                        distVec2=[((Ai(1)*(Qt(2,k)-ai(2)))-Ai(2)*(Qt(1,k)-(ai(1))))/Ai_mag, ((Bi(1)*(Qt(2,k)-bi(2)))-Bi(2)*(Qt(1,k)-(bi(1))))/Bi_mag,...
                                            ((Ci(1)*(Qt(2,k)-ci(2)))-Ci(2)*(Qt(1,k)-(ci(1))))/Ci_mag, ((Di(1)*(Qt(2,k)-di(2)))-Di(2)*(Qt(1,k)-(di(1))))/Di_mag];
                                        
                                        if distVec2(1)>=0 && distVec2(2)>=0 && distVec2(3)>=0 && distVec2(4)>=0; % Collision Case 2 (see notes)
                                            Collision=2;
                                            [minnum,minInd]=min(distVec1(:));
                                            nhatcol=([nj(1,minInd),nj(2,minInd)]'./norm([nj(1,minInd),nj(2,minInd)]));
                                            WallVec=[0,-1;1,0]*nhatcol; %vector // to wall
                                            WallVecHat=WallVec/norm(WallVec);
                                            
                                            newVel=dot(cdx(1,i).*xHati,-WallVecHat).*WallVecHat*slide+(norm(cdx(1,i)).*nhatcol)*bounce1+(rij./norm(rij))*bounce2;
                                            
                                            cx(1, i) = this.states(1, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*newVel(1);%+ this.linearVelocityCoef*this.timeStep.*this.states(4, i).*cos(this.states(3, i));
                                            cx(2, i) = this.states(2, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*newVel(2);%+ this.linearVelocityCoef*this.timeStep.*this.states(4, i).*sin(this.states(3, i));
                                            cx(3, i) = this.states(3, i) + (rand-0.5)./80;;
                                            
                                            penalty=((norm(dot(cdx(1,i).*xHati,nhatcol)))^2)./this.numAgents; 
                                            this.S=this.S+(penalty);
                                            
                                            th=cx(3,i);
                                            Pt=[[1,1,1,1].*cx(1,i);[1,1,1,1].*cx(2,i)]+[cos(th), -sin(th);sin(th),cos(th)]*[0.8,-0.8,-0.8,0.8;.9,.9,-.9,-.9].* 0.9*robotDiameter ; % Defining Points of Body i
                                            
                                            for j=1:this.numAgents; % Overlap Test
                                                if  i~=j;
                                                    thj=cx(3,j);
                                                    Qt=[[1,1,1,1].*cx(1,j);[1,1,1,1].*cx(2,j)]+[cos(thj), -sin(thj);sin(thj),cos(thj)]*[0.8,-0.8,-0.8,0.8;.9,.9,-.9,-.9].* 0.9*robotDiameter; % Defining points of body j
                                                    rij=(cx(1:2,i)-cx(1:2,j));
                                                    if norm(rij)<2.0*robotDiameter; % PHASE I
                                                        
                                                        aj=Qt(:,1); bj=Qt(:,2);cj=Qt(:,3);dj=Qt(:,4);Aj=Qt(:,2)-Qt(:,1);Bj=Qt(:,3)-Qt(:,2);Cj=Qt(:,4)-Qt(:,3);Dj=Qt(:,1)-Qt(:,4);
                                                        Aj_mag=norm(Aj);Bj_mag=norm(Bj);Cj_mag=norm(Cj);Dj_mag=norm(Dj);
                                                        ai=Pt(:,1);bi=Pt(:,2);ci=Pt(:,3);di=Pt(:,4);Ai=Pt(:,2)-Pt(:,1);Bi=Pt(:,3)-Pt(:,2);Ci=Pt(:,4)-Pt(:,3);Di=Pt(:,1)-Pt(:,4);
                                                        Ai_mag=norm(Aj); Bi_mag=norm(Bj);Ci_mag=norm(Cj);Di_mag=norm(Dj);ni=[nAj,nBj,nCj,nDj];
                                                        
                                                        % Test if any point in i is in j
                                                        for k=1:4; % PHASE2 has i entered j
                                                            distVec1=[((Aj(1)*(Pt(2,k)-aj(2)))-Aj(2)*(Pt(1,k)-(aj(1))))/Aj_mag, ((Bj(1)*(Pt(2,k)-bj(2)))-Bj(2)*(Pt(1,k)-(bj(1))))/Bj_mag,...
                                                                ((Cj(1)*(Pt(2,k)-cj(2)))-Cj(2)*(Pt(1,k)-(cj(1))))/Cj_mag, ((Dj(1)*(Pt(2,k)-dj(2)))-Dj(2)*(Pt(1,k)-(dj(1))))/Dj_mag];
                                                            distVec2=[((Ai(1)*(Qt(2,k)-ai(2)))-Ai(2)*(Qt(1,k)-(ai(1))))/Ai_mag, ((Bi(1)*(Qt(2,k)-bi(2)))-Bi(2)*(Qt(1,k)-(bi(1))))/Bi_mag,...
                                                                ((Ci(1)*(Qt(2,k)-ci(2)))-Ci(2)*(Qt(1,k)-(ci(1))))/Ci_mag, ((Di(1)*(Qt(2,k)-di(2)))-Di(2)*(Qt(1,k)-(di(1))))/Di_mag];
                                                            if distVec1(1)>=0 && distVec1(2)>=0 && distVec1(3)>=0 && distVec1(4)>=0; % Collision Case 1 (see notes)
                                                                Collision=5;
                                                            elseif distVec2(1)>=0 && distVec2(2)>=0 && distVec2(3)>=0 && distVec2(4)>=0; % Collision Case 2 (see notes)
                                                                Collision=6;
                                                            end
                                                        end
                                                        
                                                    end
                                                end
                                            end % End if j loop: every other robot tested with i
                                        end
                                    end
                                end
                            end
                        end
                    end % End if j loop: every other robot tested with i
                    
                    if Collision==0 % Iff the new velocity will not cause collision then update
                        Collision;
                        this.states(1, i) = cx(1,i);
                        this.states(2, i) = cx(2,i);
                        this.states(3, i) = cx(3,i);
                    elseif Collision==1 || Collision==2
                        this.states(1, i) = cx(1,i);
                        this.states(2, i) = cx(2,i);
                        this.states(3, i) = cx(3,i);
                    end
                    this.save();
                    this.drawRobots();
                end % Robot i's Velocity Updated Only if It will not cause a collision
                    pause(0.0005)
            
            else % Collision Simulation Turned Off
            %Vectorize update to states
            i = 1:this.numAgents;
            
            %Update velocities using unicycle dynamics 
             this.states(1,i)= this.states(1, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*cos(this.states(3, i));
             this.states(2,i)= this.states(2, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*sin(this.states(3, i));
             this.states(3,i) = this.states(3, i) + this.angularVelocityCoef*this.timeStep.*this.states(5, i);
                
            %Ensure that we're in the right range
            %this.states(3, i) = atan2(sin(this.states(3, i)) , cos(this.states(3, i)));
            this.save();
            this.drawRobots();
            pause(this.timeStep); 
            end
         end
        
        %{
        function step(this)         
                      
            %Vectorize update to states
            i = 1:this.numAgents;
            
            %Update velocities using unicycle dynamics 
            this.states(1, i) = this.states(1, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*cos(this.states(3, i));
            this.states(2, i) = this.states(2, i) + this.linearVelocityCoef*this.timeStep.*this.states(4, i).*sin(this.states(3, i));
            this.states(3, i) = this.states(3, i) + this.angularVelocityCoef*this.timeStep.*this.states(5, i);            
            
            %Ensure that we're in the right range
            this.states(3, i) = atan2(sin(this.states(3, i)) , cos(this.states(3, i)));
            
            this.save();
            this.drawRobots();
            pause(this.timeStep); 
        end
        %}
        
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

