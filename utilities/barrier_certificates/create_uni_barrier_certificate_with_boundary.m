function uni_barrier_certificate = create_uni_barrier_certificate_with_boundary(varargin)
% CREATE_UNI_BARRIER_CERTIFICATE_WITH_BOUNDARY Creates a unicycle barrier
% certificate that avoids inter-robot collisions, obstacle collisions, and
% keeps all robots inside a rectangular boundary.
%
%   Unicycle inputs are projected into the single-integrator domain via a
%   virtual point, made safe by CREATE_SI_BARRIER_CERTIFICATE_WITH_BOUNDARY,
%   then mapped back to unicycle (linear/angular velocity) inputs.
%
%   Optional named parameters
%   -------------------------
%   SafetyRadius        (default 0.12)               : min center-to-center distance (m)
%   BarrierGain         (default 150)                : higher values allow faster approach
%   ProjectionDistance  (default 0.03)               : virtual SI point offset (m)
%   VelocityMagnitudeLimit (default 0.2)             : max SI velocity magnitude (m/s)
%   BoundaryPoints      (default [-1.6 1.6 -1.0 1.0]): rectangle [xmin xmax ymin ymax]
%
%   Example
%   -------
%       bc = create_uni_barrier_certificate_with_boundary('SafetyRadius', 0.15);
%       dxu_safe = bc(dxu_desired, x);
%       dxu_safe = bc(dxu_desired, x, obstacles);  % with obstacle points (2xM)

    parser = inputParser;
    parser.addParameter('SafetyRadius',           0.12);
    parser.addParameter('BarrierGain',            150);
    parser.addParameter('ProjectionDistance',     0.03);
    parser.addParameter('VelocityMagnitudeLimit', 0.2);
    parser.addParameter('BoundaryPoints',         [-1.6, 1.6, -1.0, 1.0]);
    parse(parser, varargin{:});

    safety_radius            = parser.Results.SafetyRadius;
    gamma                    = parser.Results.BarrierGain;
    projection_distance      = parser.Results.ProjectionDistance;
    velocity_magnitude_limit = parser.Results.VelocityMagnitudeLimit;
    boundary_points          = parser.Results.BoundaryPoints;

    assert(numel(boundary_points) == 4, ...
        'BoundaryPoints must be [xmin, xmax, ymin, ymax].');
    assert(boundary_points(2) > boundary_points(1), ...
        'BoundaryPoints: xmax must be greater than xmin.');
    assert(boundary_points(4) > boundary_points(3), ...
        'BoundaryPoints: ymax must be greater than ymin.');

    % Build uni <-> SI mapping utilities
    [si_uni_dyn, uni_si_states] = create_si_to_uni_mapping('ProjectionDistance', projection_distance);
    uni_si_dyn = create_uni_to_si_mapping('ProjectionDistance', projection_distance);

    % Build the SI barrier certificate. The effective safety radius is
    % widened by 2*projection_distance to account for the virtual point
    % sitting ahead of the robot centre.
    si_barrier = create_si_barrier_certificate_with_boundary( ...
        'SafetyRadius',   safety_radius + 2*projection_distance, ...
        'BarrierGain',    gamma, ...
        'MagnitudeLimit', velocity_magnitude_limit, ...
        'BoundaryPoints', boundary_points);

    uni_barrier_certificate = @barrier_unicycle;

    function dxu = barrier_unicycle(dxu, x, obstacles)
        % BARRIER_UNICYCLE  Minimally modifies dxu to ensure safety.
        %
        %   dxu = barrier_unicycle(dxu, x)
        %   dxu = barrier_unicycle(dxu, x, obstacles)
        %
        %   Inputs
        %   ------
        %   dxu       : 2xN desired unicycle inputs [v; omega]
        %   x         : 3xN unicycle states [x; y; theta]
        %   obstacles : (optional) 2xM matrix of obstacle point positions
        %
        %   Output
        %   ------
        %   dxu : 2xN collision-free, boundary-respecting unicycle inputs

        if nargin < 3
            obstacles = [];
        end

        N = size(dxu, 2);

        if N < 2
            return
        end

        % 1. Map unicycle states/inputs -> single-integrator domain
        xi  = uni_si_states(x);     % 2xN projected positions
        dxi = uni_si_dyn(dxu, x);   % 2xN projected velocities

        % 2. Augment SI positions with obstacle positions so the SI
        %    certificate can treat them as additional boundary constraints.
        %    Obstacles are appended as extra "robots" with zero velocity,
        %    then stripped from the output.
        if ~isempty(obstacles)
            num_obs     = size(obstacles, 2);
            xi_aug      = [xi, obstacles];
            dxi_aug     = [dxi, zeros(2, num_obs)];
        else
            xi_aug  = xi;
            dxi_aug = dxi;
        end

        % 3. Apply SI barrier certificate for collision and boundary safety
        lastwarn('');
        prev_state = warning('off', 'create_si_barrier_certificate_with_boundary:QPFailed');
        dxi_aug = si_barrier(dxi_aug, xi_aug);
        [~, triggered_id] = lastwarn();
        warning(prev_state);

        if strcmp(triggered_id, 'create_si_barrier_certificate_with_boundary:QPFailed')
            warning('create_uni_barrier_certificate_with_boundary:QPFailed', ...
                'create_uni_barrier_certificate_with_boundary: Safety QP failed. Commanding zero velocities.');
            dxu = zeros(2, N);
            return
        end

        % 4. Strip obstacle columns and map safe SI velocities back to unicycle
        dxi = dxi_aug(:, 1:N);
        dxu = si_uni_dyn(dxi, x);
    end
end