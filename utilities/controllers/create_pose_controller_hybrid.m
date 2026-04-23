function pose_controller = create_pose_controller_hybrid(varargin)
% CREATE_POSE_CONTROLLER_HYBRID Creates a unicycle pose controller based on
% a two-phase hybrid control strategy.
%
%   Returns a function handle  f(states, poses) -> dxu  that drives N
%   unicycle-modeled robots to desired poses (position + orientation) using
%   a sequential two-phase approach:
%
%     Phase 1 — APPROACH: drive toward the goal position using a
%       single-integrator-to-unicycle mapping.  The robot stays in this
%       phase until it is within PositionError of the goal.
%
%     Phase 2 — ROTATE: once close enough, rotate in place until the
%       heading error is within RotationError.  If the robot drifts back
%       beyond PositionError during rotation it re-enters Phase 1.
%
%   This contrasts with the CLF controller (create_pose_controller_CLF)
%   which drives position and orientation simultaneously.  The hybrid
%   controller is simpler to tune and more predictable near the goal, at
%   the cost of not being smooth at the phase transition.
%
%   The approach state (1 = approaching, 0 = rotating) for each robot is
%   stored internally between calls.  It can also be passed in and out
%   explicitly for stateless use (e.g. when running multiple independent
%   controller instances in the same loop).
%
%   Optional named parameters
%   -------------------------
%   LinearVelocityGain    (default 0.8)  : gain passed to SI-to-uni mapping
%   AngularVelocityLimit  (default pi)   : angular velocity cap (rad/s)
%   VelocityMagnitudeLimit (default 0.15): max SI velocity magnitude (m/s)
%   PositionError         (default 0.03) : distance threshold to switch
%                                          from Phase 1 to Phase 2 (m)
%   PositionEpsilon       (default 0.01) : hysteresis band around
%                                          PositionError to prevent
%                                          chattering at the phase boundary
%   RotationError         (default 0.05) : heading threshold to declare
%                                          convergence (rad)
%
%   Example
%   -------
%       ctrl = create_pose_controller_hybrid();
%       dxu  = ctrl(states, poses);
%
%       % Stateless form — manage approach_state externally:
%       [dxu, approach_state] = ctrl(states, poses, approach_state);

    parser = inputParser;
    parser.addParameter('LinearVelocityGain',    0.8);
    parser.addParameter('AngularVelocityLimit',  pi);
    parser.addParameter('VelocityMagnitudeLimit', 0.15);
    parser.addParameter('PositionError',         0.03);
    parser.addParameter('PositionEpsilon',       0.01);
    parser.addParameter('RotationError',         0.05);
    parse(parser, varargin{:});

    lin_vel_gain  = parser.Results.LinearVelocityGain;
    ang_vel_limit = parser.Results.AngularVelocityLimit;
    vel_mag_limit = parser.Results.VelocityMagnitudeLimit;
    pos_err       = parser.Results.PositionError;
    pos_eps       = parser.Results.PositionEpsilon;
    rot_err       = parser.Results.RotationError;

    % Single-integrator-to-unicycle mapping used during the approach phase
    si_to_uni = create_si_to_uni_dynamics( ...
        'LinearVelocityGain',   lin_vel_gain, ...
        'AngularVelocityLimit', ang_vel_limit);

    % Internal approach state: 1 = approaching (Phase 1), 0 = rotating (Phase 2).
    % Initialised lazily on the first call so the controller handles any N.
    approach_state = [];

    pose_controller = @controller_hybrid;

    function [dxu, out_approach_state] = controller_hybrid(states, poses, in_approach_state)
        % CONTROLLER_HYBRID  Computes hybrid pose inputs toward desired poses.
        %
        %   [dxu]                        = controller_hybrid(states, poses)
        %   [dxu, out_approach_state]    = controller_hybrid(states, poses)
        %   [dxu, out_approach_state]    = controller_hybrid(states, poses, in_approach_state)
        %
        %   Inputs
        %   ------
        %   states           : 3xN current unicycle states  [x; y; theta]
        %   poses            : 3xN desired poses             [x; y; theta]
        %   in_approach_state: (optional) 1xN phase vector from a prior call
        %                      (1 = approach, 0 = rotate).  When omitted the
        %                      internally stored state is used.
        %
        %   Outputs
        %   -------
        %   dxu              : 2xN unicycle inputs  [v; omega]
        %   out_approach_state: 1xN phase vector after this time step

        N = size(states, 2);

        % Accept an externally supplied approach state, or initialise to
        % Phase 1 (approaching) on the very first call
        if nargin > 2
            approach_state = in_approach_state;
        elseif isempty(approach_state)
            approach_state = ones(1, N);
        end

        dxu = zeros(2, N);

        for i = 1:N
            pos_vec  = poses(1:2,i) - states(1:2,i);   % 2D position error vector
            pos_dist = norm(pos_vec);                   % scalar distance to goal

            % Wrap heading error to (-pi, pi]
            heading_err = atan2(sin(poses(3,i) - states(3,i)), ...
                                cos(poses(3,i) - states(3,i)));

            if approach_state(i) && pos_dist > pos_err - pos_eps
                % ---- Phase 1: APPROACH ----
                % Drive toward the goal position using the SI-to-uni mapping.
                % Clamp the SI velocity to VelocityMagnitudeLimit so the
                % robot does not overshoot at high gains.
                if pos_dist > vel_mag_limit
                    pos_vec = vel_mag_limit * pos_vec / pos_dist;
                end
                dxu(:,i) = si_to_uni(pos_vec, states(:,i));

            elseif abs(heading_err) > rot_err
                % ---- Phase 2: ROTATE ----
                % Position is close enough; rotate in place to match heading.
                % Use proportional angular control (gain = 2).
                % Re-enter Phase 1 if the robot has drifted beyond PositionError.
                approach_state(i) = pos_dist > pos_err;
                dxu(1,i) = 0;
                dxu(2,i) = 2 * heading_err;

            else
                % ---- CONVERGED ----
                % Both position and heading are within tolerance.
                dxu(:,i) = zeros(2,1);
            end
        end

        if nargout > 1
            out_approach_state = approach_state;
        end
    end
end
