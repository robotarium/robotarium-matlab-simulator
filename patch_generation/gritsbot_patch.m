function [ patch_data ] = gritsbot_patch()
%GRITSBOT_PATCH This is a helper function to generate patches for the
%simulated GRITSbots.  YOU SHOULD NEVER HAVE TO USE THIS FUNCTION.

    % Make it facing 0 rads

    robot_width = 0.03;
    robot_height = 0.03; 
    wheel_width = 0.005; 
    wheel_height = 0.01; 
    led_size = 0.005; 
    
    rectangle = @(w, h) [w/2 h/2 1; -w/2 h/2 1; -w/2 -h/2 1; w/2 -h/2 1];
    shift = @(r, x, y) r + repmat([x, y, 0], size(r, 1), 1);
    
    body = rectangle(robot_width, robot_height);
    wheel = rectangle(wheel_width, wheel_height);
    led = rectangle(led_size, led_size);
    
    left_wheel = shift(wheel, -(robot_width + wheel_width)/2, 0);
    right_wheel = shift(wheel, (robot_width + wheel_width)/2, 0);
    left_led = shift(led,  -robot_width/4, -robot_height/4);
    right_led = shift(led,  robot_width/4, -robot_height/4);
    
    vertices = [
     body ; 
     left_wheel; 
     right_wheel;
     left_led;
     right_led
    ];

    colors = [
     0.5*randn(1, 3) + [238, 138, 17]/255; 
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0
    ];

    colors = min(colors, 0.9);
    colors = max(colors, 0);
    
    faces = repmat([1 2 3 4 1], 5, 1);
    
    for i = 2:5
       faces(i, :) = faces(i, :) + (i-1)*4;
    end
    
   patch_data = []; 
   patch_data.vertices = vertices;
   patch_data.colors = colors;
   patch_data.faces = faces;
end

