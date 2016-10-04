%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize and record data 
%using the Robotarium simulator
%Paul Glotfelter 
%3/24/2016

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
r.step();

% Set the number of times we want to initialize the agents
iterations = 20;

% Set up some variables to store data
initial_conditions_c = cell(iterations);
minimum_distance = [];

% Set some parameters for use with the barrier certificates.  We don't want
% our agents to collide
safety = 0.06;
lambda = 0.03;

% Create a barrier certificate for use with the above parameters 
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', safety, ... 
    'ProjectionDistance', lambda);

for iteration = 1:iterations
        
    %Get randomized initial conditions 
    initial_conditions = generate_initial_conditions(N);
    
    % Sto
    initial_conditions_c{iteration} = initial_conditions;
    
    args = {'PositionError', 0.01, 'RotationError', 0.1};
    init_checker = create_is_initialized(args{:});
    automatic_parker = create_automatic_parking_controller(args{:});

    while(~init_checker(x, initial_conditions))

        x = r.get_poses();
        dxu = automatic_parker(x, initial_conditions);
        dxu = unicycle_barrier_certificate(dxu, x);      

        r.set_velocities(1:N, dxu);
        r.step();   
        
        % Determine minimum distance and save it for plotting
    
        min = 1000;         

        for i = 1:N 
           for j = 1:N
              if((i ~= j) && norm(x(1:2, i) - x(1:2, j)) < min)
                 min = norm(x(1:2, i) - x(1:2, j));
              end
           end
        end

        minimum_distance = [minimum_distance min];
    end
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

figure 
hold on 
plot(minimum_distance, 'LineWidth', 3) 
plot(repmat(safety, 1, length(minimum_distance)), 'r--', 'LineWidth', 3)
hold off
xlabel('Iterations') 
ylabel('Minimum Pariwise Distance (m)')

