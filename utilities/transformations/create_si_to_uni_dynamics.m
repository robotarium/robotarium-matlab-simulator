function si_to_uni_dyn = create_si_to_uni_dynamics(varargin)
% CREATE_SI_TO_UNI_DYNAMICS  Create a mapping from single-integrator to
% unicycle dynamics.
%
%   Returns a function handle  f(dxi, states) -> dxu  that converts 2xN
%   single-integrator velocities into 2xN unicycle velocities [v; omega]
%   using a projection-based approach.
%
%   Optional named parameters
%   -------------------------
%   LinearVelocityGain  (default 1)  : scales the linear velocity output
%   AngularVelocityLimit (default pi) : clamps angular velocity to [-avl, avl] (rad/s)
%
%   Example
%   -------
%       si_to_uni = create_si_to_uni_dynamics('LinearVelocityGain', 0.5);
%       dxu = si_to_uni(dxi, states);

    parser = inputParser;
    parser.addParameter('LinearVelocityGain',   1);
    parser.addParameter('AngularVelocityLimit', pi);
    parse(parser, varargin{:});

    lvg = parser.Results.LinearVelocityGain;
    avl = parser.Results.AngularVelocityLimit;

    si_to_uni_dyn = @si_to_uni;

    function dxu = si_to_uni(dxi, states)
        % SI_TO_UNI  Map single-integrator velocities to unicycle commands.
        %
        %   dxu = si_to_uni(dxi, states)
        %
        %   Inputs
        %   ------
        %   dxi    : 2xN single-integrator velocities  [vx; vy]
        %   states : 3xN unicycle states               [x; y; theta]
        %
        %   Output
        %   ------
        %   dxu : 2xN unicycle commands  [v; omega]

        [M, N]             = size(dxi);
        [M_states, N_states] = size(states);

        assert(M == 2, ...
            'dxi must be 2xN. Got %dx%d.', M, N);
        assert(M_states == 3, ...
            'states must be 3xN. Got %dx%d.', M_states, N_states);
        assert(N == N_states, ...
            'dxi and states must have the same number of columns (%d vs %d).', N, N_states);

        dxu = zeros(2, N);
        for i = 1:N
            h = states(3, i);
            e_fwd  = [ cos(h)  sin(h)];   % unit vector along heading
            e_perp = [-sin(h)  cos(h)];   % unit vector perpendicular (left)

            % Linear velocity: projection of dxi onto the heading direction.
            dxu(1, i) = lvg * (e_fwd * dxi(:, i));

            % Angular velocity: proportional to the angle between dxi and
            % the heading, normalised so that a 90-degree error maps to avl.
            dxu(2, i) = avl * atan2(e_perp * dxi(:, i), e_fwd * dxi(:, i)) / (pi/2);
        end
    end
end
