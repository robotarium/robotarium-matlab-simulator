function [ patch_data ] = gritsbot_patch(num_patches)
%GRITSBOT_PATCH This is a helper function to generate patches for the
%simulated GRITSbots.  YOU SHOULD NEVER HAVE TO USE THIS FUNCTION.

    robotDiameter = 0.03;

    % Scale factor (max. value of single Gaussian)
    scaleFactor = 0.5;

    % Define custom patch variables
    gritsBotBase =  [ ...
        -sqrt(2)/2 -sqrt(2)/2  1;
        sqrt(2)/2 -sqrt(2)/2  1;
        sqrt(2)/2  sqrt(2)/2  1;
        -sqrt(2)/2  sqrt(2)/2  1];
    gritsBotBase(:,1:2) = gritsBotBase(:,1:2)*robotDiameter;
    gritsBotWheel =  [ ...
        -sqrt(2)/2 -sqrt(2)/2  1;
        sqrt(2)/2 -sqrt(2)/2  1;
        sqrt(2)/2  sqrt(2)/2  1;
        -sqrt(2)/2  sqrt(2)/2  1];
    gritsBotWheel(:,1:2) = gritsBotWheel(:,1:2)...
        *diag([ robotDiameter/3 robotDiameter/6]);
    gritsBotLeftWheel =  bsxfun(@plus,gritsBotWheel,[ 0  7/6*sqrt(2)/2*robotDiameter 0 ]);
    gritsBotRightWheel = bsxfun(@plus,gritsBotWheel,[ 0 -7/6*sqrt(2)/2*robotDiameter 0 ]);
    gritsBotTailPinAngle = (pi*8/9:pi/18:pi*10/9)';
    gritsBotTailPin = [ ...
        -sqrt(2)/2 sin(gritsBotTailPinAngle(1)) 1;
        cos(gritsBotTailPinAngle) sin(gritsBotTailPinAngle) ones(size(gritsBotTailPinAngle));
        0.95*cos(gritsBotTailPinAngle(end:-1:1)) 0.95*sin(gritsBotTailPinAngle(end:-1:1)) ones(size(gritsBotTailPinAngle));
        -sqrt(2)/2 0.95*sin(gritsBotTailPinAngle(1)) 1];
    gritsBotTailPin(:,1:2) = gritsBotTailPin(:,1:2)*robotDiameter;
    %gritsBotBaseColor = [ 0.5843    0.1294    0.9647 ];
    %gritsBotBaseColor = [rand() rand() rand()];
    gritsBotWheelColor = [ 0.0 0.0 0.0 ];
    gritsBotTailPinColor = [ 0.8 0.8 0.8];
    gritsBotTagWhite = [ 1 1 1 ];
    gritsBotTagBlack = [ 0 0 0 ];

    % Define an unit circle circumscribed rectangle
    rectangleBox = [ ...
        -sqrt(2)/2 -sqrt(2)/2;
        sqrt(2)/2 -sqrt(2)/2;
        sqrt(2)/2  sqrt(2)/2;
        -sqrt(2)/2  sqrt(2)/2];
    arucoTagScale = 0.8;
    rectangleBox = rectangleBox*arucoTagScale*robotDiameter;
    arucoBoxScale = 0.15;
    arucoBoxShiftScale = 1*arucoBoxScale;
    gritsBotTag =  [ ...
        % Outer white border
        [rectangleBox ones(4,1)];
        % Inner black border
        [0.9*rectangleBox ones(4,1)];
        % Begin 4x4 aruco tag grid
        % First Row
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,1
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,2
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,3
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1) 3*rectangleBox(1,2)]) ones(4,1)]; % Grid: 1,4
        % Second Row
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,1
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,2
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,3
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1)   rectangleBox(1,2)]) ones(4,1)]; % Grid: 2,4
        % Third Row
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,1
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,2
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,3
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1)   rectangleBox(3,2)]) ones(4,1)]; % Grid: 3,4
        % Fourth Row
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(1,1) 3*rectangleBox(3,2)]) ones(4,1)]; % Grid: 4,1
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(1,1) 3*rectangleBox(3,2)]) ones(4,1)]; % Grid: 4,2
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[  rectangleBox(2,1) 3*rectangleBox(3,2)]) ones(4,1)]; % Grid: 4,3
        [bsxfun(@plus,arucoBoxScale*rectangleBox,arucoBoxShiftScale*[3*rectangleBox(2,1) 3*rectangleBox(3,2)]) ones(4,1)]];% Grid: 4,4

    % Define common patch variables
    robot_body = [gritsBotLeftWheel;gritsBotRightWheel;gritsBotTailPin;gritsBotBase;gritsBotTag];

    robot_faces =  [...
        1:size(gritsBotWheel,1)  NaN(1,8-size(gritsBotBase,1)+size(gritsBotWheel,1));
        1+size(gritsBotWheel,1):2*size(gritsBotWheel,1) NaN(1,4-size(gritsBotBase,1)+2*size(gritsBotWheel,1));
        1+2*size(gritsBotWheel,1):2*size(gritsBotWheel,1)+size(gritsBotTailPin,1);
        1+2*size(gritsBotWheel,1)+size(gritsBotTailPin,1):2*size(gritsBotWheel,1)+size(gritsBotTailPin,1)+size(gritsBotBase,1) NaN(1,12-size(gritsBotBase,1));
        %
        [reshape(bsxfun(@plus,(1:18*size(rectangleBox,1)),size(gritsBotBase,1)+2*size(gritsBotWheel,1)+size(gritsBotTailPin,1)),[],18)' NaN(18,8)]];

    
    patch_data = cell(1, num_patches); 
    
    for i = 1:num_patches
        gritsBotBaseColor = [rand() rand() rand()];
        robot_color = [gritsBotWheelColor;gritsBotWheelColor; ...
            gritsBotTailPinColor;gritsBotBaseColor;
            gritsBotTagWhite;gritsBotTagBlack;
            bsxfun(@times,ones(16,3),(rand(16,1)>0.5))];
        
        data.robot_body = robot_body;
        data.robot_face = robot_faces;
        data.robot_color = robot_color;
        patch_data{i} = data;
    end
end

