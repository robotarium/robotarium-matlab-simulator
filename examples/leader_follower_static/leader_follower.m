%% Leader-follower with static topology
% Paul Glotfelter
% 3/24/2016

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 5000;

%% Set up the Robotarium object

N = 4;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

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
desired_distance = 0.3;

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.5, 'AngularVelocityLimit', 15);
% Single-integrator barrier certificates
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.22);
% Single-integrator position controller
si_pos_controller = create_si_position_controller();

waypoints = 0.85*[1 1; -1 1; -1 -1; 1 -1]';
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
            dxi(:, 1) = si_pos_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 2;
            end
        case 2
            dxi(:, 1) = si_pos_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 3;
            end
        case 3
            dxi(:, 1) = si_pos_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 4;
            end
        case 4
            dxi(:, 1) = si_pos_controller(x(1:2, 1), waypoint);
            if(norm(x(1:2, 1) - waypoint) < close_enough)
                state = 1;
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

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();