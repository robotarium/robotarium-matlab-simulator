function pose_controller = create_pose_parking_controller_clf(varargin)
% CREATE_POSE_PARKING_CONTROLLER_CLF Wraps create_pose_controller_clf
% to zero out commands for robots that have already reached their goal pose.
%
%   Returns a function handle  f(states, poses) -> dxu  that behaves
%   identically to create_pose_controller_clf, except that any robot
%   already within PositionError and RotationError of its goal is commanded
%   to stop (zero velocity) rather than continuing to issue small corrective
%   inputs.
%
%   This is useful when robots should hold position after convergence rather
%   than oscillating around the goal under the CLF law.
%
%   Optional named parameters
%   -------------------------
%   ApproachAngleGain  (default 1)    : passed to create_pose_controller_clf
%   DesiredAngleGain   (default 2.7)  : passed to create_pose_controller_clf
%   RotationErrorGain  (default 1)    : passed to create_pose_controller_clf
%   WheelVelocityLimit (default 10.0) : passed to create_pose_controller_clf
%   WheelRadius        (default 0.016): passed to create_pose_controller_clf
%   BaseLength         (default 0.105): passed to create_pose_controller_clf
%   PositionError      (default 0.05) : distance threshold for convergence (m)
%   RotationError      (default 0.2)  : heading threshold for convergence (rad)
%
%   Example
%   -------
%       ctrl = create_pose_parking_controller_clf();
%       dxu  = ctrl(states, poses);

    parser = inputParser;
    parser.addParameter('ApproachAngleGain',  1);
    parser.addParameter('DesiredAngleGain',   2.7);
    parser.addParameter('RotationErrorGain',  1);
    parser.addParameter('WheelVelocityLimit', 10.0);
    parser.addParameter('WheelRadius',        0.016);
    parser.addParameter('BaseLength',         0.105);
    parser.addParameter('PositionError',      0.05);
    parser.addParameter('RotationError',      0.2);
    parse(parser, varargin{:});

    pos_err = parser.Results.PositionError;
    rot_err = parser.Results.RotationError;

    % Underlying CLF controller — forwards all relevant parameters
    clf_controller = create_pose_controller_clf( ...
        'ApproachAngleGain',  parser.Results.ApproachAngleGain, ...
        'DesiredAngleGain',   parser.Results.DesiredAngleGain, ...
        'RotationErrorGain',  parser.Results.RotationErrorGain, ...
        'WheelVelocityLimit', parser.Results.WheelVelocityLimit, ...
        'WheelRadius',        parser.Results.WheelRadius, ...
        'BaseLength',         parser.Results.BaseLength);

    pose_controller = @controller_clf_parking;

    function dxu = controller_clf_parking(states, poses)
        % CONTROLLER_CLF_PARKING  CLF inputs with zeroing for converged robots.
        %
        %   dxu = controller_clf_parking(states, poses)
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

        % Compute CLF inputs for all robots
        dxu = clf_controller(states, poses);

        % Identify converged robots and zero their commands
        pos_converged = vecnorm(states(1:2,:) - poses(1:2,:)) < pos_err;
        rot_converged = abs(atan2(sin(states(3,:) - poses(3,:)), ...
                                  cos(states(3,:) - poses(3,:)))) < rot_err;
        converged = pos_converged & rot_converged;

        dxu(:, converged) = 0;
    end
end
