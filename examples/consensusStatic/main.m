%Vanilla consensus with a static, undirected topology
%Paul Glotfelter 
%3/24/2016

%After running the init script, we can import some of the Robotarium's
%utilities 

import graph.*
import controllers.*
import transformations.*

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = r.getAvailableAgents(); 

% Initialize the Robotarium object with the desired number of agents
r.initialize(N);

r.setSaveParameters('hi.mat', 100, 50);

% Generate a cyclic graph Laplacian from our handy utilities.  For this
% algorithm, any connected graph will yield consensus
L = cycleGL(N); 

% Gain for the diffeomorphism transformation between single-integrator and
% unicycle dynamics
diffeomorphismGain = 0.25;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 100000;

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    %tic
    x = r.getPoses();
    %toc
    x_ = x(1, :); 
    y_ = x(2, :);
    %%% ALGORITHM %%%
    
    % See the documentation (LINK HERE) for the math that generated this
    % section
    
    for i = 1:N
        
        % Initialize velocity to zero for each agent.  This allows us to sum
        %over agent i's neighbors
        dx(:, i) = [0 ; 0];
        
        % Get the topological neighbors of agent i based on the graph
        %Laplacian L
        neighbors = r.getTopNeighbors(i, L);
        
        % Iterate through agent i's neighbors
        for j = neighbors
            
            % For each neighbor, calculate appropriate consensus term and
            %add it to the total velocity
            dx(:, i) = dx(:, i) + (x(1:2, j) - x(1:2, i));
        end      
    end   
    
    %%% END ALGORITHM %%%   
    
    % Transform the single-integrator dynamics to unicycle dynamics using a
    % diffeomorphism, which can be found in the utilities
    
    dx = barrierCertificate(dx, x, 0.1);
    
    dx = int2uni3(dx, x, diffeomorphismGain);
    
    %dx = ones(2, N) * 0.1;
        
    % Set velocities of agents 1,...,N
    r.setVelocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    
end

