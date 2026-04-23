function distances = simulate_distance_sensors_without_robots(...
    poses, ...
    obstacles, ...
    distance_sensors_orientation, ...
    distance_sensor_range, ...
    distance_sensor_error, ...
    distance_sensor_dropout_prob, ...
    distance_sensor_outlier_prob)
    % SIMULATE_DISTANCE_SENSORS simulates distance measurements for all robots.
    % 
    % Returns:
    %   distances : (S, N) matrix where S is number of sensors and N is robots.

    N = size(poses, 2);
    M = size(distance_sensors_orientation, 2);
    num_obstacles = size(obstacles, 1);
    distances = zeros(M, N);

    for i = 1:N
        R = [cos(poses(3, i)), -sin(poses(3, i)); sin(poses(3, i)), cos(poses(3, i))];
        for j = 1:M
            start_point = R * distance_sensors_orientation(1:2, j) + poses(1:2, i);
            end_point = start_point + distance_sensor_range * [cos(poses(3, i) + distance_sensors_orientation(3, j)); sin(poses(3, i) + distance_sensors_orientation(3, j))];
            ray_vector = end_point - start_point;

            min_distance = distance_sensor_range;
        
            % Check intersection with obstacles
            for k = 1:num_obstacles
                obs_start = squeeze(obstacles(k, 1, :));
                obs_end = squeeze(obstacles(k, 2, :));
                obs_vector = obs_end - obs_start;

                % Compute intersection using cross product
                r_cross_s = ray_vector(1) * obs_vector(2) - ray_vector(2) * obs_vector(1);
                if abs(r_cross_s) < 1e-10
                    continue;
                end

                diff = obs_start - start_point;
                t = (diff(1) * obs_vector(2) - diff(2) * obs_vector(1)) / r_cross_s;
                u = (diff(1) * ray_vector(2) - diff(2) * ray_vector(1)) / r_cross_s;

                if 0 <= t && t <= 1 && 0 <= u && u <= 1
                    intersection_point = start_point + t * ray_vector;
                    distance = norm(intersection_point - start_point);
                    min_distance = min(min_distance, distance);
                end
            end

            % Apply sensor noise and dropout/outlier model
            if min_distance < distance_sensor_range
                % Mixture model for noise, outliers, and dropouts
                if rand() < distance_sensor_dropout_prob
                    distances(j, i) = -1;
                elseif rand() < distance_sensor_outlier_prob
                    distances(j, i) = rand() * distance_sensor_range;
                else
                    noise = distance_sensor_error * min_distance * randn();
                    distances(j, i) = min(max(min_distance + noise, 0), distance_sensor_range);
                end
            else
                distances(j, i) = -1;
            end
        end 
    end
end