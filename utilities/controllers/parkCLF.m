function [ dxu ] = parkCLF( states, poses , K1, K3)
%PARKICLF Provides velocities to drive the robot to a desired orientation
%and position
%  

    N = size(states, 2);  
    R = @(theta) [cos(theta) sin(theta) ; -sin(theta) cos(theta)];   
            
    states(3, :) = states(3, :);
        
    for i = 1:N
       %states(3, i)  = atan(sin(states(3, i)) / cos(states(3, i)));
    end
       
    %CONVERT TO LOCAL COORDINATES % 
    
    polar = states - poses; 
    
    for i = 1:N               
        %Rotate coordinates into desired frame
        polar(1:2, i) = R(poses(3, i))*polar(1:2, i);     
        %polar(3, i) = atan(sin(polar(3, i)) / cos(polar(3, i)));
    end    
      
    % CONVERT TO POLAR COORDINATES % 
    
    for i = 1:N
        phi = polar(3, i);
        polar(3, i) = atan2(-polar(2, i), -polar(1, i));
        polar(1, i) = norm(polar(1:2, i)); 
        polar(2, i) = polar(3, i) - phi;
    end
    
    %polar
    
    % CALCULATE VELOCITIES VIA CLF
    
    dxu = zeros(2, N); 
    %(x(2) + cos(x(2))*sin(x(2))*(1 + x(3)/(x(1)*x(2))));

    for i = 1:N
        dxu(1, i) = K1 * cos(polar(2, i)) * polar(1, i);
        dxu(2, i) = K1 * polar(2, i) + K1 * (cos(polar(2, i))*sin(polar(2, i)))/(polar(2, i)) * (polar(2, i) + K3*polar(3, i));
    end
end

