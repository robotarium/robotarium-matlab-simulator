% UNI_BARRIERS
% Demonstrates the unicycle barrier certificate on N robots.
% Robots are arranged on an ellipse and repeatedly swap to the
% positions on the opposite side of the ellipse, with the barrier 
% certificate preventing collisions and boundary violations throughout.
%
% Compared to si_barriers.m, the barrier certificate is applied after
% the single-integrator to unicycle mapping rather than before, operating
% directly in unicycle space.
%
% Sean Wilson
% 07/2019

%% =========================================================
%  SIMULATION PARAMETERS
%  =========================================================

N = 10;          % Number of robots
iterations = 3000; % Total simulation steps

%% =========================================================
%  ROBOTARIUM INITIALIZATION
%  =========================================================

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%% =========================================================
%  CONTROLLER AND SAFETY SETUP
%  =========================================================

% Single-integrator position controller drives robots toward goal positions
position_control = create_si_position_controller();

% Maps single-integrator velocities to unicycle commands
si_to_uni_dyn = create_si_to_uni_dynamics();

% Barrier certificate prevents collisions and boundary violations.
% Applied after the unicycle mapping, unlike si_barriers.m.
uni_barrier_certificate = create_uni_barrier_certificate_with_boundary();

%% =========================================================
%  GOAL INITIALIZATION
%  =========================================================

% Distribute 2N points evenly around an ellipse. Robots alternate between
% the first N points and the antipodal N points, causing them to
% repeatedly swap positions across the ellipse.
xybound = [-1, 1, -0.8, 0.8];
p_theta = (1:2:2*N) / (2*N) * 2*pi;
p_circ = [xybound(2)*cos(p_theta),  xybound(2)*cos(p_theta + pi); ...
          xybound(4)*sin(p_theta),  xybound(4)*sin(p_theta + pi)];

% Initialize velocity vector for agents
dx = zeros(2, N);

% flag = 0: robots target first N ellipse points
% flag = 1: robots target antipodal N ellipse points
flag = 0;
x_goal = p_circ(:, 1:N);

%% =========================================================
%  MAIN SIMULATION LOOP
%  =========================================================

for t = 1:iterations

    % Retrieve the most recent poses from the Robotarium.
    % The time delay is approximately 0.033 seconds.
    x = r.get_poses();
    x_temp = x(1:2, :);

    % ---------------------------------------------------------
    % Check if all robots are close enough to their goals,
    % then flip the flag to swap to the antipodal goal set
    % ---------------------------------------------------------
    if norm(x_goal - x_temp, 1) < 0.03
        flag = 1 - flag;
    end

    if flag == 0
        x_goal = p_circ(:, 1:N);
    else
        x_goal = p_circ(:, N+1:2*N);
    end

    % ---------------------------------------------------------
    % Compute control inputs and apply safety
    % ---------------------------------------------------------

    % Drive robots toward their current goal positions using a single
    % integrator position controller.
    dxi = position_control(x_temp, x_goal);

    % Convert single-integrator to unicycle velocities before
    % applying the barrier certificate (unicycle space)
    dxu = si_to_uni_dyn(dxi, x);

    % Apply barrier certificate to prevent collisions and boundary violations
    dxu = uni_barrier_certificate(dxu, x);

    r.set_velocities(1:N, dxu);
    r.step();
end

%% =========================================================
%  DEBUG REPORT
%  =========================================================

% Prints collision, boundary, and actuator limit error counts.
% Fix any reported errors before submitting to the Robotarium.
r.debug();