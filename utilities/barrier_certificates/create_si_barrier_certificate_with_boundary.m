function si_barrier_certificate = create_si_barrier_certificate_with_boundary(varargin)
% CREATE_SI_BARRIER_CERTIFICATE_WITH_BOUNDARY Creates a single-integrator
% barrier certificate that avoids inter-robot collisions and keeps all
% robots inside a rectangular boundary.
%
%   Returns a function handle  f(dxi, x) -> dx  that modifies the
%   requested single-integrator velocities dxi (2xN) as little as possible
%   while guaranteeing safety.
%
%   Optional named parameters
%   -------------------------
%   SafetyRadius    (default 0.15)                  : min center-to-center distance (m)
%   BarrierGain     (default 100)                   : higher values allow faster approach
%   MagnitudeLimit  (default 0.2)                   : max input velocity magnitude (m/s)
%   BoundaryPoints  (default [-1.7, 1.7, -1.1, 1.1]): rectangle [xmin xmax ymin ymax]
%
%   Example
%   -------
%       bc = create_si_barrier_certificate_with_boundary('SafetyRadius', 0.2);
%       dx_safe = bc(dx_desired, x);

    parser = inputParser;
    parser.addParameter('SafetyRadius',   0.15);
    parser.addParameter('BarrierGain',    100);
    parser.addParameter('MagnitudeLimit', 0.2);
    parser.addParameter('BoundaryPoints', [-1.7, 1.7, -1.1, 1.1]);
    parse(parser, varargin{:});

    safety_radius   = parser.Results.SafetyRadius;
    gamma           = parser.Results.BarrierGain;
    magnitude_limit = parser.Results.MagnitudeLimit;
    boundary_points = parser.Results.BoundaryPoints;

    assert(numel(boundary_points) == 4, ...
        'BoundaryPoints must be [xmin, xmax, ymin, ymax].');
    assert(boundary_points(2) > boundary_points(1), ...
        'BoundaryPoints: xmax must be greater than xmin.');
    assert(boundary_points(4) > boundary_points(3), ...
        'BoundaryPoints: ymax must be greater than ymin.');

    qp_opts = optimoptions(@quadprog, 'Display', 'off');

    si_barrier_certificate = @barrier_certificate;

    function dx = barrier_certificate(dxi, x)
        % BARRIER_CERTIFICATE  Minimally modifies dxi to ensure safety.
        %
        %   dx = barrier_certificate(dxi, x)
        %
        %   Inputs
        %   ------
        %   dxi : 2xN desired velocities
        %   x   : 2xN (or 3xN) robot positions  (only rows 1-2 are used)
        %
        %   Output
        %   ------
        %   dx  : 2xN collision-free, boundary-respecting velocities

        N = size(dxi, 2);

        if N < 2
            dx = dxi;
            return
        end

        x = x(1:2, :);

        % Build QP inequality constraints  A*v <= b
        %   Robot-robot pairs: one constraint per unique (i,j)
        %     dh/dt >= -gamma * h^3,  h = ||xi - xj||^2 - r^2
        %   Boundary walls: four constraints per robot (x, y)
        %     dh/dt >= -0.4*gamma * h^3,  h = (wall - r/2) - coordinate
        num_pairs       = nchoosek(N, 2);
        num_constraints = num_pairs + 4*N + 8*N;
        A = zeros(num_constraints, 2*N);
        b = zeros(num_constraints, 1);

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

        row = pair;
        for k = 1:N
            cols = (2*k-1):(2*k);

            % +Y wall
            A(row, cols) = [0,  1];
            b(row)       = 0.4*gamma * (boundary_points(4) - safety_radius/2 - x(2,k))^3;
            row = row + 1;

            % -Y wall
            A(row, cols) = [0, -1];
            b(row)       = 0.4*gamma * (x(2,k) - boundary_points(3) - safety_radius/2)^3;
            row = row + 1;

            % +X wall
            A(row, cols) = [ 1, 0];
            b(row)       = 0.4*gamma * (boundary_points(2) - safety_radius/2 - x(1,k))^3;
            row = row + 1;

            % -X wall
            A(row, cols) = [-1, 0];
            b(row)       = 0.4*gamma * (x(1,k) - boundary_points(1) - safety_radius/2)^3;
            row = row + 1;
        end

        % Build Magnitude Constraints (8-sided approximation of the l2-norm)
        constraint = row;
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

        if exitflag ~= 1
            warning('create_si_barrier_certificate_with_boundary:QPFailed', ...
                'create_si_barrier_certificate_with_boundary: QP failed (exitflag %d). Commanding zero velocities.', exitflag);
            dx = zeros(2, N);
        else
            dx = reshape(vnew, 2, N);
        end
    end
end