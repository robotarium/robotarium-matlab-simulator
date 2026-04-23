function poses = generate_initial_poses(N, width, height)
    % Generate N initial poses for the robots starting from where they come off of the chargers
    if nargin < 2, width = 3.0; end
    if nargin < 3, height = 1.0; end

    xs = linspace(-width/2, width/2, 11);

    poses = [];
    for x = xs
        for y = [-height/2, height/2]
            if y < 0
                theta = pi/2;
            else
                theta = -pi/2;
            end
            poses = [poses; x, y, theta];
        end
    end

    % Randomly sample N rows without replacement
    idx = randperm(size(poses, 1), N);
    poses = poses(idx, :).';
end