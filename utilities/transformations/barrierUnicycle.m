function [ dx ] = barrierUnicycle(dxu, x, safetyRadius, lambda, varargin)
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
        
        persistent parser

        parser = inputParser;
        parser.addParameter('BarrierGain', 3);
        parse(parser, varargin{:})
        
        
        N = size(dxu, 2);
        opts = optimoptions('quadprog','Display','off');       
        gamma = parser.Results.BarrierGain;
        
        
        %Shift to single integrator
        xi = x(1:2, :) + lambda*[cos(x(3, :)) ; sin(x(3, :))];
        
        dxi = uni2int(dxu, x, lambda);   
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        num_constraints = nchoosek(N, 2);
        A = zeros(num_constraints, 2*N);
        b = zeros(num_constraints, 1);
        count = 1;
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(xi(:,i)-xi(:,j))^2-(safetyRadius + 2*lambda)^2;
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
        dx = int2uni(reshape(vnew, 2, N), x, lambda);

end

