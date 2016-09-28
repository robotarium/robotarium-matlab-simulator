%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize and record data 
%using the Robotarium simulator
%Paul Glotfelter 
%3/24/2016

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = r.getAvailableAgents();

% Initialize the Robotarium object with the desired number of agents
r.initialize(N);

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.getPoses();
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
    initialConditions = generateInitialConditions(N);
    
    % Sto
    initial_conditions_c{iteration} = initialConditions;
    
    args = {'PositionError', 0.01, 'RotationError', 0.1};
    init_checker = create_is_initialized(args{:});
    automatic_parker = create_automatic_parking_controller(args{:});

    while(~init_checker(x, initialConditions))

        x = r.getPoses();
        dxu = automatic_parker(x, initialConditions);
        dxu = unicycle_barrier_certificate(dxu, x);      
        
        %dxu
        
        for i = 1:N
           if(abs(dxu(1, i)) < 0.000001)
              dxu(1, i) = double(0.0); 
           end
        end

        for i = 1:N
           if(abs(dxu(2, i)) < 0.000001)
              dxu(2, i) = double(0.0); 
           end
        end

        r.setVelocities(1:N, dxu);
        r.step();                   
    
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

figure 
hold on 
plot(minimum_distance, 'LineWidth', 3) 
plot(repmat(safety, 1, length(minimum_distance)), 'r--', 'LineWidth', 3)
hold off
xlabel('Iterations') 
ylabel('Minimum Pariwise Distance (m)')

