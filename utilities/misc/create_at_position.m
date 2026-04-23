function at_position_checker = create_at_position(varargin)
% CREATE_AT_POSITION Creates a function to check whether robots have
% reached their desired positions (no heading requirement).
%
%   Returns a function handle  f(states, positions) -> [done, idxs]  where
%   done is true when ALL robots are within threshold, and idxs is a
%   logical 1xN vector indicating which individual robots have converged.
%
%   Compatible with both single-integrator (2xN) and unicycle (3xN) states
%   — only the first two rows are used.
%
%   Optional named parameters
%   -------------------------
%   PositionError (default 0.01) : distance threshold (m)
%
%   Example
%   -------
%       at_position = create_at_position('PositionError', 0.025);
%       [done, idxs] = at_position(states, goal_points);

    parser = inputParser;
    parser.addParameter('PositionError', 0.05);
    parse(parser, varargin{:});

    position_error = parser.Results.PositionError;

    at_position_checker = @check_at_position;

    function [done, idxs] = check_at_position(states, positions)
        % CHECK_AT_POSITION  Returns convergence status for each robot.
        %
        %   [done, idxs] = check_at_position(states, positions)
        %
        %   Inputs
        %   ------
        %   states    : 2xN or 3xN robot states  (only rows 1-2 are used)
        %   positions : 2xN desired positions     [x; y]
        %
        %   Outputs
        %   -------
        %   done : true if ALL robots have converged
        %   idxs : 1xN logical vector, true for each converged robot

        N = size(states, 2);

        assert(size(states, 1) >= 2, ...
            'states must be at least 2xN. Got %dx%d.', size(states, 1), size(states, 2));
        assert(size(positions, 1) == 2, ...
            'positions must be 2xN. Got %dx%d.', size(positions, 1), size(positions, 2));
        assert(size(positions, 2) == N, ...
            'states and positions must have the same number of columns (%d vs %d).', N, size(positions, 2));

        idxs = vecnorm(states(1:2,:) - positions) <= position_error;
        done = all(idxs);
    end
end
