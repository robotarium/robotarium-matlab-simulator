%% Barrier certificates for unicycle-modeled systems
%Paul Glotfelter 
%3/24/2016

%% Setup Robotarium object

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = r.getAvailableAgents(); 

iterations = 10000;

% Initialize the Robotarium object with the desired number of agents
r.initialize(N);

%% Set up constants for experiments

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

% Distribute the agents into a circle that fits into the Robotarium
% boundaries
xybound = [-0.5, 0.5, -0.3, 0.3];
p_theta = (1:2:2*N)/(2*N)*2*pi;
p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];
x_goal = p_circ(:,1:N);
flag = 0; %flag of task completion

lambda = 0.05;
safety = 0.05;

%% Tools to map single-integrator -> unicycle

% Get the tools we need to map from single-integrator
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', lambda);
uni_barrier_cert = create_uni_barrier_certificate('SafetyRadius', 0.03, 'ProjectionDistance', lambda);

si_pos_controller = create_si_position_controller();

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.getPoses();
    
    %% Algorithm
  
    % nominal controller, go2goal
    if norm(x_goal-x(1:2, :),1)<0.1
         flag = 1-flag;
    end
    
    if flag == 0
        x_goal = p_circ(:,1:N);
    else
        x_goal = p_circ(:,N+1:2*N);
    end
    
    % Convert to single-integrator domain 
    x_int = uni_to_si_states(x);
    
    %Currently in integrator dynamics
    dx = si_pos_controller(x_int, x_goal);
    
    % Threshold velocities for safety
    dxmax = 0.1;
    for i = 1:N
        if norm(dx(:,i)) > dxmax
            dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
        end
    end
    
    % Map to unicycle dynamics
    dx = si_to_uni_dyn(dx, x);    
    
    %Ensure the robots don't collide
    dx = uni_barrier_cert(dx, x);    
    
    % Set velocities of agents 1,...,N
    r.setVelocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    
end

