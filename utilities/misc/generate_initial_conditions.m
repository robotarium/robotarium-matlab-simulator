function [ poses ] = generate_initial_conditions(N, varargin)
    % GENERATE_INITIAL_CONDITIONS generate random poses in a rectangle
    % while ensuring a minimum spacing between each pose.
    %
    %   generate_initial_conditions(5) generates 3x5 matrix of random poses
    %   with default spacing and rectangular dimensions.
    %
    %   generate_initial_conditions(5, 'Spacing', 0.2, 'Width', 3.2, 'Height', 2)
    %   allows customizing the spacing and the rectangle dimensions.
    %
    %   Output:
    %       poses - 3 x N matrix: [x; y; theta] with theta in [-pi, pi]
    
        poses = zeros(3, N);
        
        parser = inputParser;
        parser.addParameter('Spacing', 0.3);
        parser.addParameter('Width', 3.0);
        parser.addParameter('Height', 1.8);
        parse(parser, varargin{:});
        
        spacing = parser.Results.Spacing;
        width = parser.Results.Width;
        height = parser.Results.Height;
    
        % === Feasibility Check ===
        approx_max_points = floor((width * height) / (spacing^2));
        if N > approx_max_points
            error(['Cannot fit %d points with spacing %.2f into %.2fm x %.2fm area. ' ...
                   'Maximum possible (approx.): %d.'], N, spacing, width, height, approx_max_points);
        end
    
        % === Rejection Sampling ===
        points = [];
        max_attempts = 200;
        attempts = 0;
    
        while size(points, 1) < N && attempts < max_attempts
            candidate = [(rand - 0.5) * width, (rand - 0.5) * height];
            
            if isempty(points)
                points = candidate;
            else
                dists = sqrt(sum((points - candidate).^2, 2));
                if all(dists >= spacing)
                    points = [points; candidate];
                end
            end
            attempts = attempts + 1;
        end
    
        if size(points, 1) < N
            error('Could not generate enough poses with the given spacing. Try lowering N or Spacing.');
        end
    
        % Assign x, y
        poses(1:2, :) = points';
    
        % Assign random theta between -pi and pi
        poses(3, :) = (rand(1, N) * 2 * pi) - pi;
    end