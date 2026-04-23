function [si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping(varargin)
% CREATE_SI_TO_UNI_MAPPING  Create a paired dynamics and state mapping from
% single-integrator to unicycle systems using a forward projection point.
%
%   Returns two function handles:
%
%     si_to_uni_dyn    : f(dxi, states) -> dxu
%         Converts 2xN single-integrator velocities to 2xN unicycle commands
%         [v; omega] by keeping a virtual point projected ahead of each robot.
%
%     uni_to_si_states : f(states) -> xi
%         Maps 3xN unicycle poses to 2xN single-integrator positions by
%         projecting ProjectionDistance ahead along the heading.
%
%   The single-integrator control law must be evaluated at the projected
%   positions returned by uni_to_si_states, not at the unicycle origins.
%
%   Optional named parameters
%   -------------------------
%   ProjectionDistance (default 0.05) : distance (m) the SI point is
%                                       projected ahead of the unicycle
%
%   Example
%   -------
%       [si_to_uni, uni_to_si] = create_si_to_uni_mapping('ProjectionDistance', 0.05);
%       xi  = uni_to_si(states);       % SI positions from unicycle poses
%       dxi = si_algorithm(xi);        % SI control law
%       dxu = si_to_uni(dxi, states);  % unicycle commands

    parser = inputParser;
    parser.addParameter('ProjectionDistance', 0.05);
    parse(parser, varargin{:});

    d = parser.Results.ProjectionDistance;

    % Transformation matrix from body-frame SI velocities to [v; omega].
    % The 1/d factor on omega arises from the kinematics of the projection point.
    T = [1, 0; 0, 1/d];

    si_to_uni_dyn    = @si_to_uni;
    uni_to_si_states = @uni_to_si;

    function dxu = si_to_uni(dxi, states)
        % SI_TO_UNI  Convert SI velocities to unicycle commands.
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
            h = states(3, i);
            % Rotation matrix from world frame to body frame.
            R = [cos(h), sin(h); -sin(h), cos(h)];
            dxu(:, i) = T * R * dxi(:, i);
        end
    end

    function xi = uni_to_si(states)
        % UNI_TO_SI  Project unicycle poses to SI positions.
        %
        %   xi = uni_to_si(states)
        %
        %   Input
        %   -----
        %   states : 3xN unicycle states  [x; y; theta]
        %
        %   Output
        %   ------
        %   xi : 2xN single-integrator positions  [x; y] of the projection point

        assert(size(states, 1) == 3, ...
            'states must be 3xN. Got %dx%d.', size(states, 1), size(states, 2));

        xi = states(1:2, :) + d * [cos(states(3, :)); sin(states(3, :))];
    end
end
