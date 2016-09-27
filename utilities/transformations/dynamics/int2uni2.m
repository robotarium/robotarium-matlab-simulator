function [ dx ] = int2uni2(dxi, states, kv, kw)
%INT2UNI2 Converts single-integrator to unicycle dynamics
%   dx = INT2UNI2(dxi, states, kv, kw) 
%   dxi: single-integrator dynamics
%   states: unicycle states (3 x N)
%   kv: linear velocity gain 
%   kw: rotational velocity again.  Due to normalization, the w value will
%   be in the range -kw to kw.

    N = size(dxi, 2); 
    dx = zeros(2, N);
    
    for i = 1:N
        
        dx(1, i) = kv * [cos(states(3, i)) sin(states(3, i))] * dxi(:, i);
        
        %Normalizing the output of atan2 to between -kw and kw
        dx(2, i) = kw * atan2([-sin(states(3, i)) cos(states(3, i))]*dxi(:, i), ...
                              [cos(states(3, i)) sin(states(3, i))]*dxi(:, i))/(pi/2);
    end
    
end

