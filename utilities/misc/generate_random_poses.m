function poses = generate_random_poses(N, varargin)
% GENERATE_RANDOM_POSES  Generate random, well-spaced poses inside a rectangle.
%
%   Returns a 3xN matrix of poses [x; y; theta] whose (x,y) positions are
%   uniformly sampled inside a Width x Height rectangle centred at the origin,
%   subject to a minimum separation distance between every pair of points.
%   Headings theta are drawn uniformly from [-pi, pi).
%
%   Optional named parameters
%   -------------------------
%   Spacing (default 0.3) : minimum centre-to-centre distance between poses (m)
%   Width   (default 3.0) : rectangle width  (m)
%   Height  (default 1.8) : rectangle height (m)
%
%   Example
%   -------
%       poses = generate_random_poses(5);
%       poses = generate_random_poses(5, 'Spacing', 0.2, 'Width', 3.2, 'Height', 2.0);

    parser = inputParser;
    parser.addParameter('Spacing', 0.5);
    parser.addParameter('Width',   3.0);
    parser.addParameter('Height',  1.8);
    parse(parser, varargin{:});

    spacing = parser.Results.Spacing;
    width   = parser.Results.Width;
    height  = parser.Results.Height;

    % ── Feasibility check ────────────────────────────────────────────────────
    % Rough upper bound on how many spaced points can fit in the area.
    approx_max = floor((width * height) / spacing^2);
    assert(N <= approx_max, ...
        ['Cannot fit %d poses with spacing %.2f m into a %.2f x %.2f m area. ' ...
         'Approximate maximum: %d.'], N, spacing, width, height, approx_max);

    % ── Rejection sampling ───────────────────────────────────────────────────
    % Repeatedly draw random candidates and keep those that are at least
    % `spacing` metres away from every already-accepted point.
    accepted    = zeros(0, 2);
    max_attempts = 10000;

    for attempt = 1:max_attempts
        if size(accepted, 1) == N
            break;
        end

        candidate = [(rand - 0.5) * width, (rand - 0.5) * height];

        % Accept immediately if no points exist yet; otherwise check spacing.
        if isempty(accepted) || all(vecnorm(accepted - candidate, 2, 2) >= spacing)
            accepted = [accepted; candidate]; %#ok<AGROW>
        end
    end

    assert(size(accepted, 1) == N, ...
        ['Could not place %d poses with spacing %.2f m after %d attempts. ' ...
         'Try reducing N or Spacing.'], N, spacing, max_attempts);

    % ── Assemble output ──────────────────────────────────────────────────────
    poses        = zeros(3, N);
    poses(1:2,:) = accepted';                      % x, y  (metres)
    poses(3,:)   = rand(1, N) * 2*pi - pi;         % theta (radians, uniform in [-pi, pi))
end
