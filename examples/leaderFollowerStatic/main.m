%Leader-follower with static topology
%Paul Glotfelter 
%3/24/2016

%Run for 300 iterations
iterations = 2000;
u_h = [];

%Get Robotarium object and set the save parameters

r = Robotarium();
N = r.getAvailableAgents();

r.initialize(N);

%Graph laplacian  
followers = -completeGL(N-1); 
L = zeros(N, N); 
L(2:N, 2:N) = followers;
L(2, 2) = L(2, 2) + 1; 
L(2, 1) = -1; 

%Initialize velocity vector
dx = zeros(2, N);

%State for leader
state = 0;

%collisionAvoidanceGain = 0.001; 
formationControlGain = 10;
desiredDistance = 0.09;

for t = 1:iterations
           
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.getPoses();
    
    %%% ALGORITHM %%%
    
    for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dx(:, i) = [0 ; 0];
        
        neighbors = r.getTopNeighbors(i, L);
        
        for j = neighbors 
            dx(:, i) = dx(:, i) + ...
            formationControlGain*(norm(x(1:2, j) - x(1:2, i))^2 -  desiredDistance^2)*(x(1:2, j) - x(1:2, i));
        end      
    end   
    
    %%% END ALGORITHM %%%   
        
    % Make the leader travel between waypoints
    
    switch state 

        case 0             
            dx(:, 1) = positionInt(x(:, 1), [0.25 ; 0.15], 0.05);
            if(norm(x(1:2, 1) - [0.25 ; 0.15]) < 0.05) 
               state = 1; 
            end           
        case 1
            dx(:, 1) = positionInt(x(:, 1), [-0.25 ; 0.15], 0.05);
            if(norm(x(1:2, 1) - [-0.25 ; 0.15]) < 0.05) 
               state = 2; 
            end
        case 2
            dx(:, 1) = positionInt(x(:, 1), [-0.25 ; -0.15], 0.05);
            if(norm(x(1:2, 1) - [-0.25 ; -0.15]) < 0.05)
               state = 3; 
            end
        case 3
            dx(:, 1) = positionInt(x(:, 1), [0.25 ; -0.15], 0.05);
            if(norm(x(1:2, 1) - [0.25 ; -0.15]) < 0.05)
               state = 0; 
            end
    end

    dx = barrierCertificate(dx, x, 0.08);
    dx = int2uni2(dx, x, 1, 2);
    
    %Set velocities 
    r.setVelocities(1:N, dx);
    
    %Iterate experiment
    r.step();
end