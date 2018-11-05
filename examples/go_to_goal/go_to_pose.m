%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular pose
%Paul Glotfelter 
%3/24/2016

N = 6;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
r.step();

% Set some parameters for use with the barrier certificates.  We don't want
% our agents to collide

% Create a barrier certificate for use with the above parameters 
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', 1.5*r.robot_diameter, ... 
    'ProjectionDistance', 0.05);
        
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, ...
    'Width', r.boundaries(2)-r.boundaries(1)-r.robot_diameter, ...
    'Height', r.boundaries(4)-r.boundaries(3)-r.robot_diameter, ...
    'Spacing', 0.5);

args = {'PositionError', 0.025, 'RotationError', 0.1};
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller(args{:});

while(~init_checker(x, initial_conditions))
    x = r.get_poses();
    dxu = automatic_parker(x, initial_conditions);
    dxu = unicycle_barrier_certificate(dxu, x);      

    r.set_velocities(1:N, dxu);
    r.step();   
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

