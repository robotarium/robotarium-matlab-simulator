function [ patch_data ] = gternal_patch()
%GTERNAL_PATCH Generates patch data for the GTernal robot used in the
%Robotarium.  YOU SHOULD NEVER HAVE TO USE THIS FUNCTION.
%
% PATCH_DATA = GTERNAL_PATCH() generates a struct containing patch data
% for a GTernal robot patch.
%
% The GTernal body is a "tombstone" shape: a rectangle with a
% semicircular arch on the top (front) and a flat base on the bottom
% (rear). Two drive wheels protrude from the sides. A single RGB LED
% is located in the front-right of the robot.
%
% Coordinate convention (body frame, facing +x / "right" on screen):
%   +x = forward (front/arch of robot),  +y = left,
%   origin = centre of the wheel axle
%
% Reference: Kim, Soobum, et al. "GTernal: A Robot Design for the 
% Autonomous Operation of a Multi-robot Research Testbed." DARS, 2024.

    % ---------------------------------------------------------------
    % Physical dimensions (metres, consistent with ARobotarium constants)
    %   robot_diameter  = 0.11 m  -> body_width  = 0.11 m
    %   base_length     = 0.105 m -> wheel axle separation
    %   wheel_radius    = 0.016 m -> rendered wheel height = 0.032 m
    %
    % Tombstone geometry:
    %   - Width       : body_width  (= robot_diameter)
    %   - Rect height : body_rect_h  (rear flat section)
    %   - Arch radius : body_width / 2  (semicircle caps the top)
    %   - Total height: body_rect_h + arch_radius
    %
    % Origin is at the centre of the wheel axle.
    %   y_axle = 0  (axle centre)
    %   rear of body at y = -wheel_radius
    %   front of body at y = -wheel_radius + body_rect_h + arch_radius
    % ---------------------------------------------------------------
    wheel_width  = 0.008;
    wheel_height = 0.032;           % = 2 * wheel_radius
    wheel_radius = wheel_height / 2;
 
    % Derived from physical constraints:
    %   outside wheel to outside wheel = 0.11 m  -> body_width = 0.11 - wheel_width
    %   back edge to front of arch     = 0.095 m -> body_rect_h = 0.095 - arch_radius
    body_width   = 0.110 - wheel_width;   % = 0.102 m
    arch_radius  = body_width / 2;        % = 0.051 m
    body_rect_h  = 0.095 - arch_radius;   % = 0.044 m

    led_half = 0.008;  % half side-length of square LED

    % y offset: shift all geometry so axle centre is at y = 0.
    % Previously rear of body was at y=0 and axle at y=wheel_radius,
    % so subtract wheel_radius from every y coordinate.
    yo = -wheel_radius;

    % ---------------------------------------------------------------
    % Helper: circle polygon, returns (n x 3) homogeneous vertices
    % ---------------------------------------------------------------
    circle_poly = @(cx, cy, r, n) ...
        [cx + r*cos(linspace(0, 2*pi, n+1)'), ...
         cy + r*sin(linspace(0, 2*pi, n+1)'), ...
         ones(n+1, 1)];

    % ---------------------------------------------------------------
    % 1. Body — tombstone outline
    %    Flat base at y=yo, straight sides, semicircular arch at top.
    % ---------------------------------------------------------------
    n_arch      = 24;
    arch_angles = linspace(0, pi, n_arch);   % right (0) to left (pi)
    arch_pts    = [arch_radius * cos(arch_angles)', ...
                   yo + body_rect_h + arch_radius * sin(arch_angles)', ...
                   ones(n_arch, 1)];

    body_verts = [
        -body_width/2,  yo,                1;    % bottom-left
         body_width/2,  yo,                1;    % bottom-right
         body_width/2,  yo + body_rect_h,  1;    % top-right rect corner
         arch_pts(2:end-1, :);                    % arch interior (no dup corners)
        -body_width/2,  yo + body_rect_h,  1;    % top-left rect corner
    ];
    n_body = size(body_verts, 1);

    % ---------------------------------------------------------------
    % 2. Left wheel — centred on axle (y=0), rear edge at y=-wheel_radius
    % ---------------------------------------------------------------
    wx_l = -(body_width/2 + wheel_width/2);
    left_wheel = [
        wx_l - wheel_width/2,   wheel_radius, 1;
        wx_l + wheel_width/2,   wheel_radius, 1;
        wx_l + wheel_width/2,  -wheel_radius, 1;
        wx_l - wheel_width/2,  -wheel_radius, 1;
    ];

    % ---------------------------------------------------------------
    % 3. Right wheel
    % ---------------------------------------------------------------
    wx_r =  (body_width/2 + wheel_width/2);
    right_wheel = [
        wx_r - wheel_width/2,   wheel_radius, 1;
        wx_r + wheel_width/2,   wheel_radius, 1;
        wx_r + wheel_width/2,  -wheel_radius, 1;
        wx_r - wheel_width/2,  -wheel_radius, 1;
    ];

    % ---------------------------------------------------------------
    % 4. Single LED — front-right of robot body.
    %    Placed at color-slot index 4, which is the slot ARobotarium's
    %    draw_robots() overwrites with the current LED RGB command via
    %    FaceVertexCData(4, :).
    % ---------------------------------------------------------------
    led_cx    =  body_width * 0.28;
    led_cy    =  yo + body_rect_h + arch_radius * 0.50;
    led_verts = [
        led_cx - led_half,  led_cy + led_half, 1;
        led_cx + led_half,  led_cy + led_half, 1;
        led_cx + led_half,  led_cy - led_half, 1;
        led_cx - led_half,  led_cy - led_half, 1;
    ];

    % ---------------------------------------------------------------
    % Assembly
    %   Part 1 : body
    %   Part 2 : left wheel
    %   Part 3 : right wheel
    %   Part 4 : LED  <-- slot 4, recolored by ARobotarium at runtime
    % ---------------------------------------------------------------
    vertices = [
        body_verts;
        left_wheel;
        right_wheel;
        led_verts;
    ];

    % ---------------------------------------------------------------
    % Colors
    % ---------------------------------------------------------------
    gt_gold   = [179, 163, 105] / 255;
    dark_grey = [0.20, 0.20, 0.20];
    led_white = [1.00, 1.00, 1.00];

    colors = [
        gt_gold;    % body
        dark_grey;  % left wheel
        dark_grey;  % right wheel
        led_white;  % LED (slot 4)
    ];

    % ---------------------------------------------------------------
    % Faces — one polygon per part, vertex indices are global
    % ---------------------------------------------------------------
    part_sizes = [n_body, 4, 4, 4];
    n_parts    = numel(part_sizes);
    n_max      = max(part_sizes);
    faces      = NaN(n_parts, n_max);
    offset     = 0;
    for p = 1:n_parts
        sz             = part_sizes(p);
        faces(p, 1:sz) = offset + (1:sz);
        offset         = offset + sz;
    end

    % ---------------------------------------------------------------
    % Output
    % ---------------------------------------------------------------
    patch_data          = [];
    patch_data.vertices = vertices;
    patch_data.colors   = colors;
    patch_data.faces    = faces;
end