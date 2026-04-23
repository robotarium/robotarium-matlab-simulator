function pose_controller = create_pose_controller_clf(varargin)
% CREATE_POSE_CONTROLLER_CLF Creates a unicycle pose controller based on a
% Control Lyapunov Function (CLF).
%
%   Returns a function handle  f(states, poses) -> dxu  that drives N
%   unicycle-modeled robots to desired poses (position + orientation)
%   simultaneously.
%
%   The CLF is defined in the robot-centric polar error coordinates
%   (e, theta, alpha) following the formulation of Aicardi et al. (1995).
%   Convergence to the desired pose is guaranteed for alpha ~= ±pi/2.
%
%   Outputs are scaled so that the implied wheel velocities never exceed
%   WheelVelocityLimit.  When the raw CLF output would require a wheel
%   speed beyond the limit, both v and omega are scaled down by the same
%   factor so the direction of the command is preserved.
%
%   Optional named parameters
%   -------------------------
%   ApproachAngleGain  (default 1)    : gamma — scales linear velocity;
%                                       higher values drive faster approach
%   DesiredAngleGain   (default 2.7)  : k — scales angular correction toward
%                                       the desired final heading
%   RotationErrorGain  (default 1)    : h — weights heading error relative
%                                       to approach angle in the angular law
%   WheelVelocityLimit (default 12.5) : maximum wheel rotational speed (rad/s)
%   WheelRadius        (default 0.016): wheel radius (m)
%   BaseLength         (default 0.105): wheel-to-wheel distance (m)
%
%   Example
%   -------
%       ctrl = create_pose_controller_clf();
%       dxu  = ctrl(states, poses);

    parser = inputParser;
    parser.addParameter('ApproachAngleGain',  1);
    parser.addParameter('DesiredAngleGain',   2.7);
    parser.addParameter('RotationErrorGain',  1);
    parser.addParameter('WheelVelocityLimit', 10.0);
    parser.addParameter('WheelRadius',        0.016);
    parser.addParameter('BaseLength',         0.105);
    parse(parser, varargin{:});

    gamma           = parser.Results.ApproachAngleGain;
    k               = parser.Results.DesiredAngleGain;
    h               = parser.Results.RotationErrorGain;
    wheel_vel_limit = parser.Results.WheelVelocityLimit;
    wheel_radius    = parser.Results.WheelRadius;
    base_length     = parser.Results.BaseLength;

    % D maps wheel velocities [w_r; w_l] -> unicycle inputs [v; omega].
    % Dinv maps [v; omega] -> [w_r; w_l], used to check wheel feasibility.
    D    = [wheel_radius/2,           wheel_radius/2; ...
           -wheel_radius/base_length, wheel_radius/base_length];
    Dinv = inv(D);

    pose_controller = @controller_clf;

    function dxu = controller_clf(states, poses)
        % CONTROLLER_CLF  Computes CLF-based unicycle inputs toward desired poses.
        %
        %   dxu = controller_CLF(states, poses)
        %
        %   Inputs
        %   ------
        %   states : 3xN current unicycle states  [x; y; theta]
        %   poses  : 3xN desired poses             [x; y; theta]
        %
        %   Output
        %   ------
        %   dxu : 2xN unicycle inputs  [v; omega]
        %         v     — linear velocity
        %         omega — angular velocity
        %
        %   The control law is derived from the polar error coordinates:
        %     e     — distance from robot to goal
        %     theta — angle to goal in the goal frame
        %     alpha — heading error relative to the line-of-sight to goal
        %
        %   Control law (Aicardi et al., 1995):
        %     v     = gamma * e * cos(alpha)
        %     omega = k*alpha + gamma*(cos(alpha)*sin(alpha)/alpha)*(alpha + h*theta)

        N = size(states, 2);

        assert(size(states, 1) == 3, ...
            'states must be 3xN. Got %dx%d.', size(states, 1), size(states, 2));
        assert(size(poses, 1) == 3, ...
            'poses must be 3xN. Got %dx%d.', size(poses, 1), size(poses, 2));
        assert(size(poses, 2) == N, ...
            'states and poses must have the same number of columns (%d vs %d).', N, size(poses, 2));

        % 2D rotation matrix: rotates a vector by angle x
        R = @(x) [cos(x), -sin(x); sin(x), cos(x)];

        dxu = zeros(2, N);

        for i = 1:N
            % Express the position error in the goal frame so that the
            % desired heading aligns with the x-axis of that frame
            pos_error = R(-poses(3,i)) * (poses(1:2,i) - states(1:2,i));

            % Polar error coordinates
            e     = norm(pos_error);                         % distance to goal
            theta = atan2(pos_error(2), pos_error(1));       % angle to goal in goal frame
            alpha = theta - (states(3,i) - poses(3,i));      % heading error
            alpha = atan2(sin(alpha), cos(alpha));            % wrap to (-pi, pi]

            ca = cos(alpha);
            sa = sin(alpha);

            % Raw CLF control law
            dxu(1,i) = gamma * e * ca;
            dxu(2,i) = k*alpha + gamma * (ca*sa/alpha) * (alpha + h*theta);

            % Scale [v; omega] down if any wheel would exceed its speed limit.
            % Converting to wheel space and taking the worst-case ratio
            % preserves the v/omega ratio (i.e. the intended turning curvature).
            wheel_vels = Dinv * dxu(:,i);
            max_wheel  = max(abs(wheel_vels));
            if max_wheel > wheel_vel_limit
                dxu(:,i) = dxu(:,i) * (wheel_vel_limit / max_wheel);
            end
        end
    end
end