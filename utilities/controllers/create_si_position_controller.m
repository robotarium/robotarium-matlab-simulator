function si_position_controller = create_si_position_controller(varargin)
% CREATE_SI_POSITION_CONTROLLER Creates a single-integrator position controller.
%
%   Returns a function handle  f(states, poses) -> dx  that drives N
%   single-integrator robots toward desired positions using proportional
%   control, with output velocities clamped to VelocityMagnitudeLimit.
%
%   Optional named parameters
%   -------------------------
%   XVelocityGain          (default 0.8)  : proportional gain on x error
%   YVelocityGain          (default 0.8)  : proportional gain on y error
%   VelocityMagnitudeLimit (default 0.15) : max output velocity magnitude (m/s)
%
%   Example
%   -------
%       ctrl = create_si_position_controller('XVelocityGain', 1, 'YVelocityGain', 1);
%       dx   = ctrl(states, poses);

    parser = inputParser;
    parser.addParameter('XVelocityGain',          0.8);
    parser.addParameter('YVelocityGain',          0.8);
    parser.addParameter('VelocityMagnitudeLimit', 0.15);
    parse(parser, varargin{:});

    gains      = diag([parser.Results.XVelocityGain; parser.Results.YVelocityGain]);
    vel_limit  = parser.Results.VelocityMagnitudeLimit;

    si_position_controller = @position_controller_si;

    function dx = position_controller_si(states, poses)
        % POSITION_CONTROLLER_SI  Proportional SI position control.
        %
        %   dx = position_controller_si(states, poses)
        %
        %   Inputs
        %   ------
        %   states : 2xN current positions
        %   poses  : 2xN desired positions
        %
        %   Output
        %   ------
        %   dx : 2xN velocity commands, clamped to VelocityMagnitudeLimit

        N = size(states, 2);

        assert(size(states, 1) == 2, ...
            'states must be 2xN. Got %dx%d.', size(states, 1), size(states, 2));
        assert(size(poses, 1) == 2, ...
            'poses must be 2xN. Got %dx%d.', size(poses, 1), size(poses, 2));
        assert(size(poses, 2) == N, ...
            'states and poses must have the same number of columns (%d vs %d).', N, size(poses, 2));

        % Proportional control: dx = K * (goal - state)
        dx = gains * (poses - states);

        % Clamp any velocity that exceeds the magnitude limit, preserving direction
        norms     = arrayfun(@(k) norm(dx(:,k)), 1:N);
        to_clamp  = norms > vel_limit;
        if any(to_clamp)
            dx(:, to_clamp) = vel_limit * dx(:, to_clamp) ./ norms(to_clamp);
        end
    end
end
