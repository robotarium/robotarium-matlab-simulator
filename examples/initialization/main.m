%Initializing the agents to random positions with barrier certificates 
%and data plotting
%Paul Glotfelter 
%3/24/2016

close all
clear all

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = r.getAvailableAgents();

% Initialize the Robotarium object with the desired number of agents
r.initialize(N);

x = r.getPoses();
r.step();

iterations = 20;

initial_conditions_c = cell(iterations);
minimum_distance = [];

safety = 0.06;
lambda = 0.03;


for iteration = 1:iterations
    
    display('On iteration') 
    iteration
    %Get randomized initial conditions 
    initialConditions = generateInitialConditions(N);
    
    initial_conditions_c{iteration} = initialConditions;
    
    args = {'PositionError', 0.01, 'RotationError', 0.1};
    init_checker = create_is_initialized(args{:});
    automatic_parker = create_automatic_parking_controller(args{:});

    while(~init_checker(x, initialConditions))

        x = r.getPoses();
        dxu = automatic_parker(x, initialConditions);
        dxu = barrierUnicycle(dxu, x, safety, lambda, 'BarrierGain', 1);      
        
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

