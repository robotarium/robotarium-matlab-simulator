function [ uni_barrier_certificate ] = create_uni_barrier_certificate(varargin)
%CREATE_UNI_BARRIER_CERTIFICATE Summary of this function goes here
%   Detailed explanation goes here

    parser = inputParser;
    parser.addParameter('BarrierGain', 3);
    parser.addParmeter('SafetyRadius', 0.05);
    parser.addParameter('ProjectionDistance', 0.05);
    parse(parser, varargin{:})
    
    opts = optimoptions('quadprog','Display','off');       
    gamma = parser.Results.BarrierGain;
    safety_radius = parser.Results.SafetyRadius;
    projection_distance = parser.Results.ProjectionDistance;
    
    uni_barrier_certificate = @(dxu, x) barrier_unicycle(dxu, x, gamma, safety_radius, projection_distance);

    function [ dxu ] = barrier_unicycle(dxu, x, gamma, safety_radius, projection_distance)
    % Wraps unicycle dynamics in safety barrier
    %certificates
    %   This function accepts unicycle dynamics and wraps them in
    %   barrier certificates to ensure that collisions do not occur. 
    %
    %   dx = BARRIERCERTIFICATE(dxi, x, safetyRadius) 
    %   dx: generated safe, single-integrator inputs
    %   dxi: single-integrator synamics 
    %   x: States of the agents 
    %   safetyRadius:  Size of the agents (or desired separation distance)        

    N = size(dxu, 2);

    %Shift to single integrator
    xi = x(1:2, :) + projection_distance*[cos(x(3, :)) ; sin(x(3, :))];

    dxi = uni2int(dxu, x, projection_distance);   

    %Generate constraints for barrier certificates based on the size of
    %the safety radius
    num_constraints = nchoosek(N, 2);
    A = zeros(num_constraints, 2*N);
    b = zeros(num_constraints, 1);
    count = 1;
    for i = 1:(N-1)
        for j = (i+1):N
            h = norm(xi(:,i)-xi(:,j))^2-(safety_radius + 2*projection_distance)^2;
            A(count, (2*i-1):(2*i)) = 2*(xi(:,i)-xi(:,j))';
            A(count, (2*j-1):(2*j)) = -2*(xi(:,i)-xi(:,j))';
            b(count) = -gamma*h;
            count = count + 1;
        end
    end

    A = -A;
    b = -b;               

    %Solve QP program generated earlier
    vhat = reshape(dxi,2*N,1);
    H = 2*eye(2*N);
    f = -2*vhat;          

    vnew = quadprog(H, double(f), A, b, [], [], [], [], [], opts);  

    %Set robot velocities to new velocities
    dxu = int2uni(reshape(vnew, 2, N), x, projection_distance);

    end
end

