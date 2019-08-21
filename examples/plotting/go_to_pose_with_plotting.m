% This is the same code as go_to_pose.m with some plotting routines
% included. The robots will be marked by colored circles which correspond 
% to colored goal markers. The iteration count and execution time will be 
% displayed in the lower left of the figure and position data of the robots 
% will be underneath them. An image of the Georgia Institute of Technology
% Logo will be projected as the background of the experiment.

% A good practice when plotting time varying animations for the 
% Robotarium is to update the data contained in the plot every loop and 
% not replot to reduce computation time. This is done in this script.
% Sean Wilson
% 07/2019

N = 6;
initial_positions = generate_initial_conditions(N, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

% How many times the loop will execute.
iterations = 5000;

% Set some parameters for use with the barrier certificates.  We don't want
% our agents to collide

% Create a barrier certificate for use with the above parameters 
unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();
        
%Get randomized final goal locations in the robotarium arena
final_goal_points = generate_initial_conditions(N, ...
    'Width', r.boundaries(2)-r.boundaries(1)-r.robot_diameter, ...
    'Height', r.boundaries(4)-r.boundaries(3)-r.robot_diameter, ...
    'Spacing', 0.5);

args = {'PositionError', 0.025, 'RotationError', 0.05};
init_checker = create_is_initialized(args{:});
controller = create_waypoint_controller(args{:});

% Get initial location data for while loop condition.
x=r.get_poses();

% Plotting Initialization
% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = rand(N,3);%{'.-k','.-b','.-r','.-g','.-m','.-y','.-c'};

%Marker, font, and line sizes
marker_size_goal = determine_marker_size(r, 0.20);
marker_size_robot = determine_robot_marker_size(r);
font_size = determine_font_size(r, 0.05);
line_width = 5;

start_time = tic; %The start time to compute time elapsed.

for i = 1:N
    % Initialize additional information plot here. Note the order of
    % plotting matters, objects that are plotted later will appear over
    % object plotted previously.
    
    % Text for robot identification
    robot_caption = sprintf('Robot %d', i);
    % Text with robot position information
    robot_details = sprintf('X-Pos: %d \nY-Pos: %d', x(1,i), x(2,i));
    % Text with goal identification
    goal_caption = sprintf('G%d', i);
    % Plot colored square for goal location.
    d(i) = plot(final_goal_points(1,i), final_goal_points(2,i),'s','MarkerSize',marker_size_goal,'LineWidth',line_width,'Color',CM(i,:));
    % Plot the arrow indicating goal orientation.
    a(i) = quiver(final_goal_points(1,i), final_goal_points(2,i), 0.12*cos(final_goal_points(3,i)), 0.12*sin(final_goal_points(3,i)), 'LineWidth', line_width, 'MaxHeadSize', 2*line_width, 'Color',CM(i,:));
    % Plot the goal identification text inside the goal location
    goal_labels{i} = text(final_goal_points(1,i)-0.05, final_goal_points(2,i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold');
    % Plot colored circles showing robot location.
    g(i) = plot(x(1,i),x(2,i),'o','MarkerSize', marker_size_robot,'LineWidth',5,'Color',CM(i,:));
    % Plot the robot label text 
    robot_labels{i} = text(500, 500, robot_caption, 'FontSize', font_size, 'FontWeight', 'bold');
    % Plot the robot position information text
    robot_details_text{i} = text(500, 500, robot_details, 'FontSize', font_size, 'FontWeight', 'bold'); 
end

% Plot the iteration and time in the lower left. Note when run on your 
% computer, this time is based on your computers simulation time. For a
% better approximation of experiment time on the Robotarium when running
% this simulation on your computer, multiply iteration by 0.033. 
iteration_caption = sprintf('Iteration %d', 0);
time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));

iteration_label = text(-1.5, -0.8, iteration_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');
time_label = text(-1.5, -0.9, time_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');

% Import and scale the GT logo appropriately.
gt_img = imread('GTLogo.png'); % Original input image file

% Display the image with an associated spatial referencing object.
x_img = linspace(-1.0, 1.0, size(gt_img,2));
y_img = linspace(1.0, -1.0, size(gt_img,1)); %Note the 1 to -1 here due to the (1,1) pixel being in the top left corner.
gt_img_handle = image(x_img, y_img, gt_img,'CDataMapping','scaled');

% We can change the order of plotting priority, we will plot goals on the 
% bottom and have the iteration/time caption on top.
uistack(a,'bottom'); %Arrows are at the very bottom.
uistack([goal_labels{:}], 'bottom'); % Goal labels are at the very bottom, arrows are now only above the goal labels.
uistack(d, 'bottom');% Goal squares are at the very bottom, goal labels are above the squares and goal arrows are above those.
uistack([iteration_label], 'top'); % Iteration label is on top.
uistack([time_label], 'top'); % Time label is above iteration label.
uistack(gt_img_handle, 'bottom'); % Image under everything else.

r.step();

for i = 1:iterations
    %Check if all robots have reached their goal location.
    if(~init_checker(x, final_goal_points))
        x = r.get_poses();
        
        % Update Plotting Information and Locations
        for q = 1:N
            g(q).XData = x(1,q);
            g(q).YData = x(2,q);
            
            robot_labels{q}.Position = x(1:2, q) + [-0.15;0.15];
            robot_details = sprintf('X-Pos: %0.2f \nY-Pos: %0.2f', x(1,q), x(2,q));
            robot_details_text{q}.String = robot_details;
            robot_details_text{q}.Position = x(1:2, q) - [0.2;0.25];
        end
        
        dxu = controller(x, final_goal_points);
        dxu = unicycle_barrier_certificate(dxu, x);      

        r.set_velocities(1:N, dxu);
        r.step();   
    else    
        % All robots have reached their goal locations, choose new ones and
        % update goal location plot. 
        
        %Get randomized final goal locations in the robotarium arena
        final_goal_points = generate_initial_conditions(N, ...
            'Width', r.boundaries(2)-r.boundaries(1)-r.robot_diameter, ...
            'Height', r.boundaries(4)-r.boundaries(3)-r.robot_diameter, ...
            'Spacing', 0.5);

        for j = 1:N
            goal_labels{j}.Position = final_goal_points(1:2, j)-[0.05;0];

            d(j).XData = final_goal_points(1,j);
            d(j).YData = final_goal_points(2,j);
            
            a(j).XData = final_goal_points(1,j);
            a(j).YData = final_goal_points(2,j);
            a(j).UData = 0.12*cos(final_goal_points(3,j));
            a(j).VData = 0.12*sin(final_goal_points(3,j));
        end
    end
    
    % Update Iteration and Time marker
    iteration_caption = sprintf('Iteration %d', i);
    time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));
    
    iteration_label.String = iteration_caption;
    time_label.String = time_caption;
    
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window 
    % does not change size).

    marker_size_goal = num2cell(ones(1,N)*determine_marker_size(r, 0.20));
    [d.MarkerSize] = marker_size_goal{:};
    marker_size_robot = num2cell(ones(1,N)*determine_robot_marker_size(r));
    [g.MarkerSize] = marker_size_robot{:};
    font_size = determine_font_size(r, 0.05);
    iteration_label.FontSize = font_size;
    time_label.FontSize = font_size;
    
    for k = 1:N
        % Have to update font in loop for some conversion reasons.
        % Again this is unnecessary when submitting as the figure
        % window does not change size when deployed on the Robotarium.
        robot_labels{k}.FontSize = font_size;
        goal_labels{k}.FontSize = font_size;
        robot_details_text{k}.FontSize = font_size;
    end
    
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

%% Helper Functions

% Marker Size Helper Function to scale size of markers for robots with figure window
% Input: robotarium class instance
function marker_size = determine_robot_marker_size(robotarium_instance)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
robot_ratio = (robotarium_instance.robot_diameter + 0.03)/...
    (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * robot_ratio;

end

% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end
