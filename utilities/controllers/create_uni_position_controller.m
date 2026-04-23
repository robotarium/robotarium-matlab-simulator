function uni_position_controller = create_uni_position_controller(varargin)
% CREATE_UNI_POSITION_CONTROLLER Creates a unicycle position controller.
%
%   Returns a function handle  f(states, poses) -> dxu  that drives N
%   unicycle-modeled robots toward desired 2D positions (no heading target).
%
%   Internally this wraps create_si_position_controller via
%   create_si_to_uni_mapping: SI positions are projected ahead of each
%   unicycle, the SI controller runs at the projected point, and the
%   resulting SI velocities are mapped back to unicycle commands [v; omega].
%
%   Optional named parameters
%   -------------------------
%   XVelocityGain          (default 0.8)  : proportional gain on x error
%   YVelocityGain          (default 0.8)  : proportional gain on y error
%   VelocityMagnitudeLimit (default 0.15) : max SI velocity magnitude (m/s)
%   ProjectionDistance     (default 0.05) : distance (m) the SI point is
%                                           projected ahead of the unicycle
%
%   Example
%   -------
%       ctrl = create_uni_position_controller('VelocityMagnitudeLimit', 0.1);
%       dxu  = ctrl(states, poses);

    parser = inputParser;
    parser.addParameter('XVelocityGain',          0.8);
    parser.addParameter('YVelocityGain',          0.8);
    parser.addParameter('VelocityMagnitudeLimit', 0.15);
    parser.addParameter('ProjectionDistance',     0.03);
    parse(parser, varargin{:});

    % ── Build the SI controller and SI<->unicycle mappings ────────────────
    si_controller = create_si_position_controller( ...
        'XVelocityGain',          parser.Results.XVelocityGain, ...
        'YVelocityGain',          parser.Results.YVelocityGain, ...
        'VelocityMagnitudeLimit', parser.Results.VelocityMagnitudeLimit);

    [si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping( ...
        'ProjectionDistance', parser.Results.ProjectionDistance);

    uni_position_controller = @position_controller_uni;

    function dxu = position_controller_uni(states, poses)
        % POSITION_CONTROLLER_UNI  SI-based unicycle position control.
        %
        %   dxu = position_controller_uni(states, poses)
        %
        %   Inputs
        %   ------
        %   states : 3xN current unicycle states  [x; y; theta]
        %   poses  : 2xN desired positions         [x; y]
        %
        %   Output
        %   ------
        %   dxu : 2xN unicycle commands  [v; omega]

        assert(size(states, 1) == 3, ...
            'states must be 3xN. Got %dx%d.', size(states, 1), size(states, 2));
        assert(size(poses, 1) == 2, ...
            'poses must be 2xN. Got %dx%d.', size(poses, 1), size(poses, 2));
        assert(size(poses, 2) == size(states, 2), ...
            'states and poses must have the same number of columns (%d vs %d).', ...
            size(states, 2), size(poses, 2));

        % Project unicycle states to SI positions (point ahead of each robot).
        xi = uni_to_si_states(states);

        % Run the SI controller at the projected positions.
        dxi = si_controller(xi, poses);

        % Map SI velocities back to unicycle commands.
        dxu = si_to_uni_dyn(dxi, states);
    end
end