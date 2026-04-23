function at_pose_checker = create_at_pose(varargin)
% CREATE_AT_POSE Creates a function to check whether unicycle robots have
% reached their desired poses (position + orientation).
%
%   Returns a function handle  f(states, poses) -> [done, idxs]  where
%   done is true when ALL robots are within threshold, and idxs is a
%   logical 1xN vector indicating which individual robots have converged.
%
%   Optional named parameters
%   -------------------------
%   PositionError (default 0.01) : distance threshold (m)
%   RotationError (default 0.05) : heading threshold (rad)
%
%   Example
%   -------
%       at_pose = create_at_pose('PositionError', 0.025, 'RotationError', 0.05);
%       [done, idxs] = at_pose(states, goal_poses);

    parser = inputParser;
    parser.addParameter('PositionError', 0.05);
    parser.addParameter('RotationError', 0.2);
    parse(parser, varargin{:});

    position_error = parser.Results.PositionError;
    rotation_error = parser.Results.RotationError;

    at_pose_checker = @check_at_pose;

    function [done, idxs] = check_at_pose(states, poses)
        % CHECK_AT_POSE  Returns convergence status for each robot.
        %
        %   [done, idxs] = check_at_pose(states, poses)
        %
        %   Inputs
        %   ------
        %   states : 3xN current unicycle states  [x; y; theta]
        %   poses  : 3xN desired poses             [x; y; theta]
        %
        %   Outputs
        %   -------
        %   done : true if ALL robots have converged
        %   idxs : 1xN logical vector, true for each converged robot

        N = size(states, 2);

        assert(size(states, 1) == 3, ...
            'states must be 3xN. Got %dx%d.', size(states, 1), size(states, 2));
        assert(size(poses, 1) == 3, ...
            'poses must be 3xN. Got %dx%d.', size(poses, 1), size(poses, 2));
        assert(size(poses, 2) == N, ...
            'states and poses must have the same number of columns (%d vs %d).', N, size(poses, 2));

        pos_converged = vecnorm(states(1:2,:) - poses(1:2,:)) <= position_error;
        rot_converged = abs(atan2(sin(states(3,:) - poses(3,:)), ...
                                  cos(states(3,:) - poses(3,:)))) <= rotation_error;
        idxs = pos_converged & rot_converged;
        done = all(idxs);
    end
end
