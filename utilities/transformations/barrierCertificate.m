function [ dx ] = barrierCertificate(dxi, x, safetyRadius, varargin)
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
        parser.addParameter('BarrierGain', 1e4);
        parser.addParameter('MaxVelocity', 0.1);
        parser.addParameter('SafetyDistance', 0.08);
        parse(parser, varargin{:})
        gamma = parser.Results.BarrierGain;
        safetyRadius = parser.Results.SafetyDistance;
        max_velocity = parser.Results.MaxVelocity;
         
        N = size(dxi, 2);
        opts = optimoptions('quadprog','Display','off');
        x = x(1:2, :);    
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        A = []; 
        b = [];
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(x(1:2,i)-x(1:2,j))^2-safetyRadius^2;
                Anew = zeros(1,2*N);
                Anew((2*i-1):(2*i)) = -2*(x(:,i)-x(:,j))';
                Anew((2*j-1):(2*j)) =  2*(x(:,i)-x(:,j))';
                A = [A ; Anew];
                b = [b ; gamma*h^3];
            end
        end                

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
        dx = reshape(vnew, 2, N);
end

