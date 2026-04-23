function ekf_experiment(varargin)
% EKF_EXPERIMENT
% Run Unicycle EKF experiment with the passed parameters. All parameters 
% have defaults if you don't want to mess with them.

    % Parse command line arguments similar to argparse
    p = inputParser;
    addParameter(p, 'total_waypoints', 10, @isnumeric);
    addParameter(p, 'process_noise', [0.0001, 0.0001, 0.0001], @isnumeric);
    addParameter(p, 'spoof_gps_measurements', true, @islogical);
    addParameter(p, 'gps_measurement_interval_distribution', [5.0, 2.5], @isnumeric);
    addParameter(p, 'gps_measurement_noise', [0.0001, 0.0001], @isnumeric);
    addParameter(p, 'use_gyro_measurements', false, @islogical);
    addParameter(p, 'gyro_measurement_noise', 0.001, @isnumeric);
    addParameter(p, 'use_orientation_measurements', false, @islogical);
    addParameter(p, 'orientation_measurement_noise', 0.001, @isnumeric);

    parse(p, varargin{:});
    args = p.Results;

    validate_inputs(args);
    
    run_ekf_experiment(args.total_waypoints, args.process_noise, ...
                       args.spoof_gps_measurements, args.gps_measurement_interval_distribution, ...
                       args.gps_measurement_noise, args.use_gyro_measurements, ...
                       args.gyro_measurement_noise, args.use_orientation_measurements, ...
                       args.orientation_measurement_noise);
end

function run_ekf_experiment(total_waypoints, process_noise, spoof_gps_measurements, ...
                            gps_measurement_interval_distribution, gps_measurement_noise, ...
                            use_gyro_measurements, gyro_measurement_noise, ...
                            use_orientation_measurements, orientation_measurement_noise)
    N = 1;
    initial_conditions = [1.0; 0.0; 0.0];
    r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, ...
                   'InitialConditions', initial_conditions);

    % Note: Standard MATLAB Robotarium uses lowercase properties
    arena_width = r.boundaries(2) - r.boundaries(1) - 4*r.robot_diameter;
    arena_height = r.boundaries(4) - r.boundaries(3) - 4*r.robot_diameter;
    goal_points = generate_random_poses(total_waypoints, 'Width', arena_width, 'Height', arena_height, 'Spacing', 0.5);

    % Create controllers and safety barriers
    unicycle_pose_controller = create_pose_parking_controller_clf();
    uni_barrier_cert = create_uni_barrier_certificate();
    at_pose = create_at_pose();

    x = r.get_poses();
    r.step();

    wheel_radius = r.wheel_radius;
    base_length = r.base_length;
    dt = r.time_step;

    % Noise setup
    counts_to_rad = 2 * pi / (28 * 100.37); % Assuming standard motor specs
    encoder_noise_std = 0.25; 
    encoder_ang_vel_var = (encoder_noise_std * counts_to_rad / dt) ^ 2;
    encoder_noise_matrix = eye(2) * encoder_ang_vel_var;
    process_noise_matrix = diag(process_noise);

    if spoof_gps_measurements
        gps_measurement_noise_matrix = diag(gps_measurement_noise);
    else
        gps_measurement_noise_matrix = [];
    end

    if use_gyro_measurements
        gyro_measurement_noise_matrix = gyro_measurement_noise;
    else
        gyro_measurement_noise_matrix = [];
    end

    if use_orientation_measurements
        orientation_measurement_noise_matrix = orientation_measurement_noise;
    else
        orientation_measurement_noise_matrix = [];
    end

    % Initialize EKF objects
    ekf = UnicycleEKF(initial_conditions(:), zeros(3, 3), base_length, wheel_radius, encoder_noise_matrix, process_noise_matrix);
    ekf_pure_predict = UnicycleEKF(initial_conditions(:), zeros(3, 3), base_length, wheel_radius, encoder_noise_matrix, process_noise_matrix);

    % Plotting Setup inside Robotarium figure
    figure(r.figure_handle);
    hold on;
    gt_trail = plot(NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    ekf_trail = plot(NaN, NaN, 'r--', 'LineWidth', 1.5, 'DisplayName', 'EKF (with updates)');
    gps_scatter = scatter(NaN, NaN, 80, 'g', 'x', 'LineWidth', 2, 'DisplayName', 'Spoofed GPS');
    legend([gt_trail, ekf_trail, gps_scatter], 'Location', 'northwest', 'FontSize', 12);
    
    % Resize event listener
    set(r.figure_handle, 'SizeChangedFcn', @(src, event) on_resize(src, r, gps_scatter));

    % History arrays
    gt_history = [];
    ekf_history = [];
    ekf_pure_predict_history = [];
    ekf_theta_history = [];
    ekf_pure_predict_theta_history = [];
    Pdiag_history = [];
    gps_measurement_history = [];
    gps_measurement_step_indices = [];  
    gyro_measurement_history = [];  
    encoder_history = [];  
    ground_truth_poses = [];  
    orientation_measurement_history = []; 

    encoders_prev = r.get_encoders();
    sim_time = 0.0;
    next_gps_measurement_time = sim_time + max(0.1, gps_measurement_interval_distribution(1) + gps_measurement_interval_distribution(2) * randn());

    for i = 1:size(goal_points, 2)
        waypoint = goal_points(:, i);
        fprintf('Next waypoint: [%f; %f; %f]\n', waypoint(1), waypoint(2), waypoint(3));
        
        at_goal = at_pose(x, waypoint);
        while isempty(at_goal) || ~at_goal(1)
            x = r.get_poses();
            encoders_curr = r.get_encoders();
            encoder_history = [encoder_history, encoders_curr(:, 1)];
            ground_truth_poses = [ground_truth_poses, x(:, 1)];

            dxu = unicycle_pose_controller(x, waypoint);
            dxu = uni_barrier_cert(dxu, x);

            sim_time = sim_time + dt;

            if use_gyro_measurements
                gyros = r.get_gyros();
                yaw_rate = gyros(3, 1);
                gyro_measurement_history = [gyro_measurement_history, yaw_rate];
                ekf.update_gyro(yaw_rate, dt, gyro_measurement_noise_matrix);
            end

            if use_orientation_measurements
                orientations = r.get_orientations();
                orientation = orientations(1);
                orientation_measurement_history = [orientation_measurement_history, orientation];
                ekf.update_orientation(deg2rad(orientation), orientation_measurement_noise_matrix);
            end

            if spoof_gps_measurements && sim_time >= next_gps_measurement_time
                gps_meas = x(1:2, 1) + sqrt(gps_measurement_noise(:)) .* randn(2, 1);
                ekf.update_gps(gps_meas, gps_measurement_noise_matrix);
                gps_measurement_step_indices = [gps_measurement_step_indices, size(gt_history, 2) + 1];
                gps_measurement_history = [gps_measurement_history, gps_meas];
                next_gps_measurement_time = sim_time + max(0.1, gps_measurement_interval_distribution(1) + gps_measurement_interval_distribution(2) * randn());
            end

            % Explicitly cast to double before subtracting so MATLAB doesn't round to 0
            delta_counts_left = double(encoders_curr(1, 1)) - double(encoders_prev(1, 1));
            delta_counts_right = double(encoders_curr(2, 1)) - double(encoders_prev(2, 1));
            
            delta_phi_L = delta_counts_left * counts_to_rad;
            delta_phi_R = delta_counts_right * counts_to_rad;
            
            v_enc = (wheel_radius / 2) * (delta_phi_R + delta_phi_L) / dt;
            w_enc = (wheel_radius / base_length) * (delta_phi_R - delta_phi_L) / dt;

            ekf.predict(v_enc, w_enc, dt);
            ekf_pure_predict.predict(v_enc, w_enc, dt);
            encoders_prev = encoders_curr;

            gt_history = [gt_history, x(1:2, 1)];
            ekf_history = [ekf_history, ekf.state(1:2)];
            ekf_pure_predict_history = [ekf_pure_predict_history, ekf_pure_predict.state(1:2)];
            ekf_theta_history = [ekf_theta_history, ekf.state(3)];
            ekf_pure_predict_theta_history = [ekf_pure_predict_theta_history, ekf_pure_predict.state(3)];
            Pdiag_history = [Pdiag_history, diag(ekf.P)];

            % Update UI Plot Handles
            set(gt_trail, 'XData', gt_history(1,:), 'YData', gt_history(2,:));
            set(ekf_trail, 'XData', ekf_history(1,:), 'YData', ekf_history(2,:));
            if ~isempty(gps_measurement_history)
                set(gps_scatter, 'XData', gps_measurement_history(1,:), 'YData', gps_measurement_history(2,:));
            end
            
            r.set_velocities(1, dxu);
            r.step();
            drawnow limitrate;
            
            at_goal = at_pose(x, waypoint);
        end
    end

    % Save results
    save('ekf_experiment_results.mat', 'gt_history', 'ekf_history', ...
         'ekf_pure_predict_history', 'Pdiag_history', 'ground_truth_poses', ...
         'orientation_measurement_history', 'encoder_history', ...
         'gyro_measurement_history', 'gps_measurement_history', ...
         'gps_measurement_step_indices', 'dt', 'initial_conditions', ...
         'wheel_radius', 'base_length', 'process_noise', 'gps_measurement_noise');
         
    r.debug();
    disp('Experiment Complete. Results saved to ekf_experiment_results.mat');
end

function on_resize(src, r, gps_scatter)
    % Scale GPS marker size when the window is resized
    pos = get(src, 'Position');
    arena_w = r.boundaries(2) - r.boundaries(1);
    gps_size = ((0.05 / arena_w) * pos(3)) ^ 2;
    if gps_size > 0
        set(gps_scatter, 'SizeData', gps_size);
    end
end

function validate_inputs(args)
    if args.total_waypoints < 1
        error('Total waypoints must be at least 1');
    end
    if length(args.process_noise) ~= 3
        error('Process noise must be an array of length 3');
    end
    if args.spoof_gps_measurements && length(args.gps_measurement_noise) ~= 2
        error('GPS measurement noise must be an array of length 2');
    end
    if args.use_gyro_measurements && args.gyro_measurement_noise <= 0
        error('Gyro measurement noise must be > 0');
    end
    if args.use_orientation_measurements && args.orientation_measurement_noise <= 0
        error('Orientation measurement noise must be > 0');
    end
end