function si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion(varargin)
% CREATE_SI_TO_UNI_DYNAMICS_WITH_BACKWARDS_MOTION  Create a mapping from
% single-integrator to unicycle dynamics that permits reverse driving.
%
%   Returns a function handle  f(dxi, states) -> dxu  that converts 2xN
%   single-integrator velocities into 2xN unicycle velocities [v; omega].
%   Unlike the standard mapping, the robot may drive backwards when doing
%   so requires less heading rotation than driving forwards.
%
%   Optional named parameters
%   -------------------------
%   LinearVelocityGain   (default 1)  : scales the linear velocity output
%   AngularVelocityLimit (default pi) : clamps angular velocity to [-avl, avl] (rad/s)
%
%   Example
%   -------
%       si_to_uni = create_si_to_uni_dynamics_with_backwards_motion();
%       dxu = si_to_uni(dxi, states);

    parser = inputParser;
    parser.addParameter('LinearVelocityGain',   1);
    parser.addParameter('AngularVelocityLimit', pi);
    parse(parser, varargin{:});

    lvg  = parser.Results.LinearVelocityGain;
    avl  = parser.Results.AngularVelocityLimit;
    wrap = @(x) atan2(sin(x), cos(x));   % wrap angle to (-pi, pi]

    si_to_uni_dyn = @si_to_uni;

    function dxu = si_to_uni(dxi, states)
        % SI_TO_UNI  Map single-integrator velocities to unicycle commands,
        % allowing reverse motion when it requires less rotation.
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

        [M, N]               = size(dxi);
        [M_states, N_states] = size(states);

        assert(M == 2, ...
            'dxi must be 2xN. Got %dx%d.', M, N);
        assert(M_states == 3, ...
            'states must be 3xN. Got %dx%d.', M_states, N_states);
        assert(N == N_states, ...
            'dxi and states must have the same number of columns (%d vs %d).', N, N_states);

        dxu = zeros(2, N);
        for i = 1:N
            % Angle between the desired velocity direction and the heading.
            angle = wrap(atan2(dxi(2, i), dxi(1, i)) - states(3, i));

            % If the error is within ±90° drive forwards; otherwise reverse.
            % For reverse, flip the effective heading by pi so that the
            % projection and angular-velocity calculations remain consistent.
            if angle > -pi/2 && angle < pi/2
                h = states(3, i);
                s = 1;
            else
                h = wrap(states(3, i) + pi);
                s = -1;
            end

            e_fwd  = [ cos(h)  sin(h)];   % unit vector along effective heading
            e_perp = [-sin(h)  cos(h)];   % unit vector perpendicular (left)

            % Linear velocity (sign applied after so projection is correct).
            v = lvg * (e_fwd * dxi(:, i));

            % Angular velocity: normalised so a 90-degree error maps to avl.
            dxu(2, i) = avl * atan2(e_perp * dxi(:, i), v) / (pi/2);
            dxu(1, i) = s * v;
        end
    end
end
