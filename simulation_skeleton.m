%% Simulator Skeleton File
% Paul Glotfelter
% 10/04/2016
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.

%% Get Robotarium object used to communicate with the robots/simulator
N = 12;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 1000;

% Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Insert your code here!
    
    % dxu = algorithm(x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end

% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();