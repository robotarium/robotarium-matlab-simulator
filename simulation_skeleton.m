%% Simulator Skeleton File
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.
% Sean Wilson
% 03/2026

%% Get Robotarium object used to communicate with the robots/simulator
N = 3;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Select the number of iterations for the experiment.  This value is
% arbitrary.
iterations = 1000;

% Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds.
    x = r.get_poses();
    
    %% Insert your algorithm here!
    % Create velocities for each robot to follow (dxu).
    % dxu must be a 2xN matrix of unicycle inputs [v; omega].
    
    % dxu = algorithm();
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Advance the simulation by one time step.  This call is required every iteration.
    r.step();
end

% Call r.debug() after the experiment to check for any safety violations
% (collisions, boundary exits, actuator limit breaches) before submitting
% to the physical Robotarium.
r.debug();