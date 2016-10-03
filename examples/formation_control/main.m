%% Formation control utilizing edge tension energy with a static, undirected
%communication topology
%Paul Glotfelter 
%3/24/2016

%% Setup Robotarium object

% Get Robotarium object used to communicate with the robots/simulator
% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 6; 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
linearVelocityGain = 1; 
angularVelocityGain = pi/2;
formationControlGain = 4;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 100000;

% Communication topology for the desired formation.  We need 2 * N - 3 = 9
% edges to ensure that the formation is rigid.
L = [3 -1 0 -1 0 -1 ; ... 
    -1 3 -1 0 -1 0 ; ... 
    0 -1 3 -1 0 -1 ; ... 
    -1 0 -1 3 -1 0 ; ... 
    0 -1 0 -1 3 -1 ; ... 
   -1 0 -1 0 -1 3];

% The desired inter-agent distance for the formation
d = 0.2; 

% Pre-compute diagonal values for the rectangular formation
ddiag = sqrt((2*d)^2 + d^2);

% Weight matrix containing the desired inter-agent distances to achieve a
% rectuangular formation
weights = [ 0 d 0 d 0 ddiag; ... 
            d 0 d 0 d 0; ... 
            0 d 0 ddiag 0 d; ... 
            d 0 ddiag 0 d 0; ... 
            0 d 0 d 0 d; ... 
            ddiag 0 d 0 d 0];
    
% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

%% Grab tools for converting to single-integrator dynamics and ensuring safety 

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.08);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ... 
    'AngularVelocityLimit', angularVelocityGain);


% Iterate for the previously specified number of iterations
for t = 0:iterations
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% ALGORITHM
    
    %This section contains the actual algorithm for formation control!
    
    %Calculate single integrator control inputs using edge-energy consensus
    for i = 1:N
        
        % Initialize velocity to zero for each agent.  This allows us to sum
        % over agent i's neighbors
        dx(:, i) = [0 ; 0];
        
        % Get the topological neighbors of agent i from the communication
        % topology
        for j = topological_neighbors(L, i)
                
            % For each neighbor, calculate appropriate formation control term and
            % add it to the total velocity

            dx(:, i) = dx(:, i) + ...
            formationControlGain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
            *(x(1:2, j) - x(1:2, i));
        end 
    end
    
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dx = si_barrier_cert(dx, x);
    dx = si_to_uni_dyn(dx, x);  
    
    % Set velocities of agents 1:N
    r.set_velocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();   
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
