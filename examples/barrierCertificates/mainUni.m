%Vanilla consensus with a static, undirected topology
%Paul Glotfelter 
%3/24/2016

%After running the init script, we can import some of the Robotarium's
%utilities 

import graph.*
import controllers.*
import transformations.*

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = r.getAvailableAgents(); 

iterations = 20000;

% Initialize the Robotarium object with the desired number of agents
r.initialize(N);

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

xybound = [-0.5, 0.5, -0.3, 0.3];
p_theta = (1:2:2*N)/(2*N)*2*pi;
p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];
x_goal = p_circ(:,1:N);
flag = 0; %flag of task completion

lambda = 0.05;
safety = 0.03;

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.getPoses();

    x_temp = x(1:2,:);
    
    %%% ALGORITHM %%%
  
    % nominal controller, go2goal
    if norm(x_goal-x_temp,1)<0.1
         flag = 1-flag;
    end
    
    if flag == 0
        x_goal = p_circ(:,1:N);
    else
        x_goal = p_circ(:,N+1:2*N);
    end
    
    
    %Use different go-to-goal
    x_int = x; 
    x_int(1:2, :) = x_int(1:2, :) + lambda*[cos(x(3, :)) ; sin(x(3, :))];
    
    %Currently in integrator dynamics
    dx = positionInt(x_int, x_goal);
    
    
    % END ALGORITHM%     
    
    dxmax = 0.1;
    for i = 1:N
        if norm(dx(:,i)) > dxmax
            dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
        end
    end
    
    dx = int2uni(dx, x, lambda);
    
    
    %Ensure the robots don't collide
    dx = barrierUnicycle(dx, x, safety, lambda);
    
    
    % Set velocities of agents 1,...,N
    r.setVelocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    
end

