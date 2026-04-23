function [uni_to_si_dyn, si_to_uni_states] = create_uni_to_si_mapping(varargin)
% CREATE_UNI_TO_SI_MAPPING  Create a paired dynamics and state mapping from
% unicycle to single-integrator systems using a forward projection point.
%
%   Returns two function handles:
%
%     uni_to_si_dyn    : f(dxu, states) -> dxi
%         Converts 2xN unicycle commands [v; omega] to 2xN single-integrator
%         velocities at the projection point.
%
%     si_to_uni_states : f(uni_states, si_states) -> xi
%         Maps 3xN unicycle poses and 2xN SI positions back to the unicycle
%         origin by reversing the projection.
%
%   Optional named parameters
%   -------------------------
%   ProjectionDistance (default 0.05) : distance (m) the SI point is
%                                       projected ahead of the unicycle
%
%   Example
%   -------
%       [uni_to_si, si_to_uni] = create_uni_to_si_mapping('ProjectionDistance', 0.05);
%       dxi = uni_to_si(dxu, states);          % SI velocities from unicycle commands
%       uni_states = si_to_uni(states, xi);    % unicycle origins from SI positions

    parser = inputParser;
    parser.addParameter('ProjectionDistance', 0.05);
    parse(parser, varargin{:});

    d = parser.Results.ProjectionDistance;

    % Inverse transformation: scales omega back to a body-frame lateral velocity.
    T = [1, 0; 0, d];

    uni_to_si_dyn    = @uni_to_si;
    si_to_uni_states = @si_to_uni;

    function dxi = uni_to_si(dxu, states)
        % UNI_TO_SI  Convert unicycle commands to SI velocities.
        %
        %   dxi = uni_to_si(dxu, states)
        %
        %   Inputs
        %   ------
        %   dxu    : 2xN unicycle commands  [v; omega]
        %   states : 3xN unicycle states    [x; y; theta]
        %
        %   Output
        %   ------
        %   dxi : 2xN single-integrator velocities  [vx; vy]

        [M, N]               = size(dxu);
        [M_states, N_states] = size(states);

        assert(M == 2, ...
            'dxu must be 2xN. Got %dx%d.', M, N);
        assert(M_states == 3, ...
            'states must be 3xN. Got %dx%d.', M_states, N_states);
        assert(N == N_states, ...
            'dxu and states must have the same number of columns (%d vs %d).', N, N_states);

        dxi = zeros(2, N);
        for i = 1:N
            h = states(3, i);
            % Rotation matrix from body frame to world frame.
            R = [cos(h), -sin(h); sin(h), cos(h)];
            dxi(:, i) = R * T * dxu(:, i);
        end
    end

    function xi = si_to_uni(uni_states, si_states)
        % SI_TO_UNI  Recover unicycle origins from SI projection positions.
        %
        %   xi = si_to_uni(uni_states, si_states)
        %
        %   Inputs
        %   ------
        %   uni_states : 3xN unicycle states          [x; y; theta]
        %   si_states  : 2xN single-integrator positions  [x; y]
        %
        %   Output
        %   ------
        %   xi : 2xN unicycle origin positions  [x; y]

        [M,    N]    = size(uni_states);
        [M_si, N_si] = size(si_states);

        assert(M == 3, ...
            'uni_states must be 3xN. Got %dx%d.', M, N);
        assert(M_si == 2, ...
            'si_states must be 2xN. Got %dx%d.', M_si, N_si);
        assert(N == N_si, ...
            'uni_states and si_states must have the same number of columns (%d vs %d).', N, N_si);

        % Subtract the projection offset to recover the unicycle origin.
        xi = si_states(1:2, :) - d * [cos(uni_states(3, :)); sin(uni_states(3, :))];
    end
end
