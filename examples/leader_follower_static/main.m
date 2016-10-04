%% Leader-follower with static topology
% Paul Glotfelter
% 3/24/2016

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 2000;

%% Set up the Robotarium object

%Get Robotarium object and set the save parameters
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents();

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

%% Create the desired Laplacian

%Graph laplacian
followers = -completeGL(N-1);
L = zeros(N, N);
L(2:N, 2:N) = followers;
L(2, 2) = L(2, 2) + 1;
L(2, 1) = -1;

%Initialize velocity vector
dxi = zeros(2, N);

%State for leader
state = 0;

% These are gains for our formation control algorithm
formation_control_gain = 10;
desired_distance = 0.09;

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 1, 'AngularVelocityLimit', 2);
% Single-integrator barrier certificates
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.08);
% Single-integrator position controller
si_pos_controller = create_si_position_controller();

for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    
    for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        neighbors = topological_neighbors(L, i);
        
        for j = neighbors
            dxi(:, i) = dxi(:, i) + ...
                formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, j) - x(1:2, i));
        end
    end
    
    %% Make the leader travel between waypoints
    
    switch state
        
        case 0
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [0.3 ; 0.2]);
            if(norm(x(1:2, 1) - [0.3 ; 0.2]) < 0.05)
                state = 1;
            end
        case 1
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [-0.3 ; 0.2]);
            if(norm(x(1:2, 1) - [-0.3 ; 0.2]) < 0.05)
                state = 2;
            end
        case 2
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [-0.3 ; -0.2]);
            if(norm(x(1:2, 1) - [-0.3 ; -0.2]) < 0.05)
                state = 3;
            end
        case 3
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [0.3 ; -0.2]);
            if(norm(x(1:2, 1) - [0.3 ; -0.2]) < 0.05)
                state = 0;
            end
    end
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxi = si_barrier_cert(dxi, x);
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %Iterate experiment
    r.step();
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();