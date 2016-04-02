%Formation control with edge-tension energies
%Paul Glotfelter
%04/01/2016

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium();

% Set the number of available agents from the Robotarium
N = 6; 

% Initialize the Robotarium object with the desired number of agents
r.initialize(N);

%Rigid communication topology.  In the plane, we need 2 * N - 3 = 9 edges
L = [3 -1 0 -1 0 -1 ; ... 
    -1 3 -1 0 -1 0 ; ... 
    0 -1 3 -1 0 -1 ; ... 
    -1 0 -1 3 -1 0 ; ... 
    0 -1 0 -1 3 -1 ; ... 
    -1 0 -1 0 -1 3];

%Specified inter-agent distance for use in edge-tension
d = 0.2; 

%Pre-computed diagonal weight entry
ddiag = sqrt((2*d)^2 + d^2);

% Symmetric weights to enforce a rectangular structure among the 6 agents. 
% Again, we need 9 edges for rigidity
weights = [ 0 d 0 d 0 ddiag; ... 
            d 0 d 0 d 0; ... 
            0 d 0 ddiag 0 d; ... 
            d 0 ddiag 0 d 0; ... 
            0 d 0 d 0 d; ... 
            ddiag 0 d 0 d 0];
    
% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively
dx = zeros(2, N);

%Gain for the formation control algorithm
formationControlGain = 4;

%Gain for the transformation from integrator to unicycle dynamics 
diffeomorphismGain = 0.25;

%Iterate for the previously specified number of iterations
for t = 0:iterations
    
    
    %Get the current robot poses
    x = r.getPoses();
    
    %%% ALGORITHM %%% 
    
    %Calculate single integrator control inputs using edge-energy consensus
    for i = 1:N
        
        dx(:, i) = [0 ; 0];
        
        for j = r.getTopNeighbors(i, L)
            
            dx(:, i) = dx(:, i) + ...
            formationControlGain* ...
            (norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2)* ... 
            (x(1:2, j) - x(1:2, i));
        
        end 
    end
    
    %%% END ALGORITHM %%%
    
    % Transform the single-integrator dynamics to unicycle dynamics using a
    % diffeomorphism, which can be found in the utilities
    dx = int2uni(dx, x, diffeomorphismGain);
        
    
    % Set velocities of agents 1,...,N
    r.setVelocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();  
end


