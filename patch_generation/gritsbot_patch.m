function [ patch_data ] = gritsbot_patch()
%GRITSBOT_PATCH This is a helper function to generate patches for the
%simulated GRITSbots.  YOU SHOULD NEVER HAVE TO USE THIS FUNCTION.
%
% PATCH_DATA = GRITSBOT_PATCH() generates a struct containing patch data
% for a robot patch.

    % Make it facing 0 rads
    robot_width = 0.11;
    robot_height = 0.1; 
    wheel_width = 0.02; 
    wheel_height = 0.04; 
    led_size = 0.01; 
    
    % Helper functions to generate vertex coordinates for a centered
    % rectangle and a helper function to shift a rectangle.
    rectangle = @(w, h) [w/2 h/2 1; -w/2 h/2 1; -w/2 -h/2 1; w/2 -h/2 1];
    shift = @(r, x, y) r + repmat([x, y, 0], size(r, 1), 1);
    
    % Create vertices for body, wheel, and led.
    body = rectangle(robot_width, robot_height);
    wheel = rectangle(wheel_width, wheel_height);
    led = rectangle(led_size, led_size);
    
    % Use pre-generated vertices and shift them around to create a robot
    left_wheel = shift(wheel, -(robot_width + wheel_width)/2, -robot_height/6);
    right_wheel = shift(wheel, (robot_width + wheel_width)/2, -robot_height/6);
    left_led = shift(led,  robot_width/8, robot_height/2 - 2*led_size);
    right_led = shift(led,  robot_width/4, robot_height/2 - 2*led_size);
    
    % Putting all the robot vertices together
    vertices = [
     body ; 
     left_wheel; 
     right_wheel;
     left_led;
     right_led
    ];

    % Only color the body of the robot.  Everything else is black.
    colors = [
     [238, 138, 17]/255; 
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0
    ];

    % This seems weird, but it basically tells the patch function which
    % vertices to connect.
    faces = repmat([1 2 3 4 1], 5, 1);
    
    for i = 2:5
       faces(i, :) = faces(i, :) + (i-1)*4;
    end
    
   patch_data = []; 
   patch_data.vertices = vertices;
   patch_data.colors = colors;
   patch_data.faces = faces;
end

