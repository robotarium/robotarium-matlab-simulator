function [ dx ] = barrierUnicycle(dxu, x, safetyRadius, lambda, varargin)
%BARRIERCERTIFICATE Wraps single-integrator dynamics in safety barrier
%certificates
%   This function accepts single-integrator dynamics and wraps them in
%   barrier certificates to ensure that collisions do not occur.  Note that
%   this algorithm bounds the magnitude of the generated output to 0.1.
%
%   dx = BARRIERCERTIFICATE(dxi, x, safetyRadius) 
%   dx: generated safe, single-integrator inputs
%   dxi: single-integrator synamics 
%   x: States of the agents 
%   safetyRadius:  Size of the agents (or desired separation distance)         
        
        persistent parser

        parser = inputParser;
        parser.addParameter('BarrierGain', 3);
        parser.addParameter('MaxVelocity', 0.1);
        parse(parser, varargin{:})
        
        
        N = size(dxu, 2);
        opts = optimoptions('quadprog','Display','off');       
        gamma = parser.Results.BarrierGain;
        max_velocity = parser.Results.MaxVelocity;
        
        
        %Shift to single integrator
        xi = x(1:2, :) + lambda*[cos(x(3, :)) ; sin(x(3, :))];
        
        dxi = uni2int(dxu, x, lambda);   
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        A = []; 
        b = [];
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(xi(:,i)-xi(:,j))^2-(safetyRadius + 2*lambda)^2;
                Anew = zeros(1,2*N);
                Anew((2*i-1):(2*i)) = 2*(xi(:,i)-xi(:,j))';
                Anew((2*j-1):(2*j)) = -2*(xi(:,i)-xi(:,j))';
                A = [A ; Anew];
                b = [b ; -gamma*h];
            end
        end
        
        A = -A;
        b = -b;               

        %Solve QP program generated earlier
        vhat = reshape(dxi,2*N,1);
        H = 2*eye(2*N);
        f = -2*vhat;     
        bounds = max_velocity*ones(2*N, 1);
        
        %If we can't solve the current problem, relax the conditions
        vnew = [];
        i = 1;
        while isempty(vnew)
            vnew = quadprog(H, double(f), A, b, [],[], -i*bounds, i*bounds, [], opts);
            i = i*2;
        end                

        %Set robot velocities to new velocities
        dx = int2uni(reshape(vnew, 2, N), x, lambda);

end

