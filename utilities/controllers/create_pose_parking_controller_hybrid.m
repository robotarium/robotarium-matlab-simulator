function pose_controller = create_pose_parking_controller_hybrid(varargin)
% CREATE_POSE_PARKING_CONTROLLER_HYBRID Wraps create_pose_controller_hybrid
% to zero out commands for robots that have already reached their goal pose.
%
%   Returns a function handle  f(states, poses) -> dxu  that behaves
%   identically to create_pose_controller_hybrid, except that any robot
%   already within PositionError and RotationError of its goal is commanded
%   to stop (zero velocity) rather than continuing to issue small corrective
%   inputs.
%
%   This is useful when robots should hold position after convergence rather
%   than chattering at the goal during the hybrid controller's rotate phase.
%
%   Optional named parameters
%   -------------------------
%   LinearVelocityGain     (default 0.8)  : passed to create_pose_controller_hybrid
%   AngularVelocityLimit   (default pi)   : passed to create_pose_controller_hybrid
%   VelocityMagnitudeLimit (default 0.15) : passed to create_pose_controller_hybrid
%   PositionError          (default 0.05) : distance threshold for convergence (m);
%                                           shared between the hybrid controller's
%                                           phase transitions and the stopping check
%   PositionEpsilon        (default 0.01) : passed to create_pose_controller_hybrid
%   RotationError          (default 0.2)  : heading threshold for convergence (rad);
%                                           shared between the hybrid controller's
%                                           phase transitions and the stopping check
%
%   Example
%   -------
%       ctrl = create_pose_controller_hybrid_with_stopping();
%       dxu  = ctrl(states, poses);

    parser = inputParser;
    parser.addParameter('LinearVelocityGain',    0.8);
    parser.addParameter('AngularVelocityLimit',  pi);
    parser.addParameter('VelocityMagnitudeLimit', 0.15);
    parser.addParameter('PositionError',         0.05);
    parser.addParameter('PositionEpsilon',       0.01);
    parser.addParameter('RotationError',         0.2);
    parse(parser, varargin{:});

    pos_err = parser.Results.PositionError;
    rot_err = parser.Results.RotationError;

    % Underlying hybrid controller — forwards all parameters so that the
    % phase-transition thresholds inside the controller stay in sync with
    % the stopping check below
    hybrid_controller = create_pose_controller_hybrid( ...
        'LinearVelocityGain',    parser.Results.LinearVelocityGain, ...
        'AngularVelocityLimit',  parser.Results.AngularVelocityLimit, ...
        'VelocityMagnitudeLimit', parser.Results.VelocityMagnitudeLimit, ...
        'PositionError',         pos_err, ...
        'PositionEpsilon',       parser.Results.PositionEpsilon, ...
        'RotationError',         rot_err);

    pose_controller = @controller_hybrid_parking;

    function dxu = controller_hybrid_parking(states, poses)
        % CONTROLLER_HYBRID_PARKING  Hybrid inputs with zeroing for converged robots.
        %
        %   dxu = controller_hybrid_parking(states, poses)
        %
        %   Inputs
        %   ------
        %   states : 3xN current unicycle states  [x; y; theta]
        %   poses  : 3xN desired poses             [x; y; theta]
        %
        %   Output
        %   ------
        %   dxu    : 2xN unicycle inputs, zeroed for any robot that has
        %            already converged to within PositionError and RotationError

        % Compute hybrid inputs for all robots
        dxu = hybrid_controller(states, poses);

        % Identify converged robots and zero their commands
        pos_converged = vecnorm(states(1:2,:) - poses(1:2,:)) < pos_err;
        rot_converged = abs(atan2(sin(states(3,:) - poses(3,:)), ...
                                  cos(states(3,:) - poses(3,:)))) < rot_err;
        converged = pos_converged & rot_converged;

        dxu(:, converged) = 0;
    end
end
