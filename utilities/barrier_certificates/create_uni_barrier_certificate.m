function [ uni_barrier_certificate ] = create_uni_barrier_certificate(varargin)
% CREATE_UNI_BARRIER_CERTIFICATE Creates a unicycle barrier certificate
% function to avoid collisions by internally using a single-integrator
% barrier certificate.
%
%   The unicycle control inputs are projected into the single-integrator
%   domain, made safe via CREATE_SI_BARRIER_CERTIFICATE, and then mapped
%   back to unicycle dynamics.
%
%   Args:
%       BarrierGain, optional: How quickly robots can approach each other
%       SafetyRadius, optional: How far apart centers of robots should remain
%       ProjectionDistance, optional: How far ahead to project a virtual
%           single integrator
%       VelocityMagnitudeLimit, optional: The maximum velocity for the
%           virtual single integrator
%
%   Returns:
%       A barrier certificate function (2xN, 3xN) -> 2xN representing the
%       barrier certificate
%
%   CREATE_UNI_BARRIER_CERTIFICATE('BarrierGain', bg)
%
%   CREATE_UNI_BARRIER_CERTIFICATE('SafetyRadius', sr)
%
%   CREATE_UNI_BARRIER_CERTIFICATE('SafetyRadius', sr, 'BarrierGain', bg)
%
%   Example:
%       bc = create_uni_barrier_certificate('SafetyRadius', 0.2)
%
%   Notes:
%       SafetyRadius should be a positive double
%       BarrierGain should be a positive double
%       In practice, the value for SafetyRadius should be a little more than
%       double the size of the robots.

    parser = inputParser;
    addOptional(parser, 'BarrierGain', 100);
    addOptional(parser, 'SafetyRadius', 0.15);
    addOptional(parser, 'ProjectionDistance', 0.05);
    addOptional(parser, 'VelocityMagnitudeLimit', 0.2);
    parse(parser, varargin{:})

    gamma                  = parser.Results.BarrierGain;
    safety_radius          = parser.Results.SafetyRadius;
    projection_distance    = parser.Results.ProjectionDistance;
    velocity_magnitude_limit = parser.Results.VelocityMagnitudeLimit;

    % Build uni <-> si mapping utilities
    [si_uni_dyn, uni_si_states] = create_si_to_uni_mapping('ProjectionDistance', projection_distance);
    uni_si_dyn = create_uni_to_si_mapping('ProjectionDistance', projection_distance);

    % Build the SI barrier certificate, accounting for the extra clearance
    % needed by the projection point offset from the robot centre
    si_barrier = create_si_barrier_certificate( ...
        'BarrierGain',   gamma, ...
        'SafetyRadius',  safety_radius + 2*projection_distance);

    uni_barrier_certificate = @barrier_unicycle;

    function [ dxu ] = barrier_unicycle(dxu, x)
        % BARRIER_UNICYCLE The parameterized barrier function
        %
        %   Args:
        %       dxu: 2xN vector of unicycle control inputs
        %       x:   3xN vector of unicycle states
        %
        %   Returns:
        %       A 2xN matrix of safe unicycle control inputs

        N = size(dxu, 2);

        if N < 2
            return
        end

        % 1. Map unicycle states/inputs -> single-integrator domain
        xi  = uni_si_states(x);          % 2xN projected positions
        dxi = uni_si_dyn(dxu, x);        % 2xN projected velocities

        % 2. Apply SI barrier certificate for collision safety.
        %    Suppress the SI-level warning and re-issue a friendlier one here
        %    so users are not exposed to internal implementation details.
        lastwarn('');                                        % clear last warning
        prev_state = warning('off', 'create_si_barrier_certificate:QPFailed');
        dxi = si_barrier(dxi, xi);
        [~, triggered_id] = lastwarn();
        warning(prev_state);                                 % restore warning state

        if strcmp(triggered_id, 'create_si_barrier_certificate:QPFailed')
            warning('create_uni_barrier_certificate:QPFailed', ...
                'create_uni_barrier_certificate: Safety QP failed. Commanding zero velocities.');
            dxu = zeros(2, N);
            return
        end

        % 3. Map safe SI velocities back to unicycle inputs
        dxu = si_uni_dyn(dxi, x);
    end
end