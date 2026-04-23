function si_barrier_certificate = create_si_barrier_certificate(varargin)
% CREATE_SI_BARRIER_CERTIFICATE Creates a single-integrator barrier
% certificate to avoid inter-robot collisions.
%
%   Returns a function handle  f(dxi, x) -> dx  that modifies the
%   requested single-integrator velocities dxi (2xN) as little as possible
%   while guaranteeing that no two robots violate the safety radius.
%
%   Optional named parameters
%   -------------------------
%   SafetyRadius    (default 0.15) : minimum center-to-center distance (m)
%   BarrierGain     (default 100)  : higher values allow faster approach
%   MagnitudeLimit  (default 0.2)  : max input velocity magnitude (m/s)
%
%   Example
%   -------
%       bc = create_si_barrier_certificate('SafetyRadius', 0.2);
%       dx_safe = bc(dx_desired, x);

    parser = inputParser;
    parser.addParameter('SafetyRadius',   0.15);
    parser.addParameter('BarrierGain',    100);
    parser.addParameter('MagnitudeLimit', 0.2);
    parse(parser, varargin{:});

    safety_radius   = parser.Results.SafetyRadius;
    gamma           = parser.Results.BarrierGain;
    magnitude_limit = parser.Results.MagnitudeLimit;
    qp_opts       = optimoptions(@quadprog, 'Display', 'off');

    si_barrier_certificate = @barrier_certificate;

    function dx = barrier_certificate(dxi, x)
        % BARRIER_CERTIFICATE  Minimally modifies dxi to ensure safety.
        %
        %   dx = barrier_certificate(dxi, x)
        %
        %   Inputs
        %   ------
        %   dxi : 2xN desired velocities (clamped to MagnitudeLimit before solving)
        %   x   : 2xN (or 3xN) robot positions  (only rows 1-2 are used)
        %
        %   Output
        %   ------
        %   dx  : 2xN collision-free velocities

        N = size(dxi, 2);

        if N < 2
            dx = dxi;
            return
        end

        % Clamp input magnitudes before solving
        norms        = arrayfun(@(k) norm(dxi(:, k)), 1:N);
        to_clamp     = norms > magnitude_limit;
        if any(to_clamp)
            dxi(:, to_clamp) = magnitude_limit * dxi(:, to_clamp) ./ norms(to_clamp);
        end

        x = x(1:2, :);

        % Build QP inequality constraints  A*v <= b
        %   One constraint per unique pair (i,j): dh/dt >= -gamma * h^3
        %   where h = ||xi - xj||^2 - r^2
        num_pairs = nchoosek(N, 2) + 8*N;
        A = zeros(num_pairs, 2*N);
        b = zeros(num_pairs, 1);

        % Build Robot Constraints
        pair = 1;
        for i = 1:(N-1)
            for j = (i+1):N
                diff = x(:,i) - x(:,j);
                h    = dot(diff, diff) - safety_radius^2;

                A(pair, (2*i-1):(2*i)) = -2*diff';
                A(pair, (2*j-1):(2*j)) =  2*diff';
                b(pair)                =  gamma * h^3;

                pair = pair + 1;
            end
        end

        % Build Magnitude Constraints (8-sided approximation of the l2-norm)
        constraint = pair;
        for i = 1:N
            % vx <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [1, 0];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;

            % 1/sqrt(2) * (vx + vy) <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [1/sqrt(2), 1/sqrt(2)];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;

            % vy <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [0, 1];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;

            % 1/sqrt(2) * (-vx + vy) <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [-1/sqrt(2), 1/sqrt(2)];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;

            % -vx <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [-1, 0];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;

            % 1/sqrt(2) * (-vx - vy) <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [-1/sqrt(2), -1/sqrt(2)];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;

            % -vy <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [0, -1];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;

            % 1/sqrt(2) * (vx - vy) <= magnitude_limit
            A(constraint, (2*i-1):(2*i)) = [1/sqrt(2), -1/sqrt(2)];
            b(constraint) = magnitude_limit * cos(pi/8);
            constraint = constraint + 1;
        end

        % Solve: min  ||v - dxi||^2  s.t.  A*v <= b
        vhat = dxi(:);
        [vnew, ~, exitflag] = quadprog(sparse(2*eye(2*N)), -2*vhat, A, b, [], [], [], [], [], qp_opts);

        % exitflag == 1 is the only flag that guarantees a certified optimum.
        % Anything else (infeasible, unbounded, iteration limit, etc.) means
        % we cannot trust the solution.  In that case we stop all robots
        % rather than commanding an unsafe velocity.
        if exitflag ~= 1
            warning('create_si_barrier_certificate:QPFailed', ...
                'create_si_barrier_certificate: QP failed (exitflag %d). Commanding zero velocities.', exitflag);
            dx = zeros(2, N);
        else
            dx = reshape(vnew, 2, N);
        end
    end
end