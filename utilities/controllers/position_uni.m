function [ dx ] = position_uni(states, poses)
%POSITIONCLF Utilizes a Controlled Lyapunov Function (CLF) to drive a
%unicycle system to a desired position
%  This function operates on unicycle states and desired poses and returns
% a unicycle-velocity-valued vector.

    [M, N] = size(states);
    [M_poses, N_poses] = size(poses);
    
    assert(M == 3, 'Row size of states vector must be 3!  Given size is %i', M);
    assert(M_poses, 'Row size of desired poses (%i) must be 2!', M_poses);
    assert(N == N_poses, 'Row size of states vector (%i) must be row size of desired poses (%i)', N, N_poses);
    
    dxu = zeros(2, N);

    for i = 1:N
        
        pos_error = poses(:, i) - states(1:2, i);
        rot_error = atan2(dy, dx
        
        dx_ = poses(1, i) - states(1, i);
        dy = poses(2, i) - states(2, i);
        dt = atan2(dy, dx_);

        dist = sqrt(dx_^2 + dy^2);

        dxu(1, i) = dist*cos(dt - states(3, i));
        dxu(2, i) = dist*sin(dt - states(3, i));
    end 
end

