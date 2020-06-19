%% Leader-follower with static topology
% Paul Glotfelter edited by Sean Wilson
% 07/2019

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 5000;

%% Set up the Robotarium object

N = 4;
initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

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
state = 1;

% These are gains for our formation control algorithm
formation_control_gain = 10;
desired_distance = 0.2;

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
% Single-integrator position controller
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.1);

waypoints = [-1 0.8; -1 -0.8; 1 -0.8; 1 0.8]';
close_enough = 0.05;

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
    
    waypoint = waypoints(:, state);
    
    switch state        
        case 1
            dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 2;
            end
        case 2
            dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 3;
            end
        case 3
            dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 4;
            end
        case 4
            dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 1;
            end
    end
    
        
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %Iterate experiment
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();