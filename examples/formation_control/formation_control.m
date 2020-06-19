%% Formation control utilizing edge tension energy with a static, undirected
% communication topology
% Paul Glotfelter updated by Sean Wilson
% 07/2019

%% Set up Robotarium object

N = 6;
initial_conditions = generate_initial_conditions(N, 'Width', 2, 'Height', 1, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
formation_control_gain = 10;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 2000;

% Communication topology for the desired formation.  We need 2 * N - 3 = 9
% edges to ensure that the formation is rigid.
L = [3 -1 0 -1 0 -1 ; ... 
    -1 3 -1 0 -1 0 ; ... 
    0 -1 3 -1 0 -1 ; ... 
    -1 0 -1 3 -1 0 ; ... 
    0 -1 0 -1 3 -1 ; ... 
   -1 0 -1 0 -1 3];

% The desired inter-agent distance for the formation
d = 0.4; 

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

uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.5, 'AngularVelocityLimit', pi/2);

% Iterate for the previously specified number of iterations
for t = 0:iterations
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    
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
            formation_control_gain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
            *(x(1:2, j) - x(1:2, i));
        end 
    end
    
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dx
    norms = arrayfun(@(x) norm(dx(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);
    
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dx = si_to_uni_dyn(dx, x);  
    dx = uni_barrier_cert(dx, x);
    
    % Set velocities of agents 1:N
    r.set_velocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();   
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
