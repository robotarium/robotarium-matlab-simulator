function [ dx ] = barrierUnicycle(dxu, x, safetyRadius, lambda)
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
         
        N = size(dxu, 2);
        opts = optimoptions('quadprog','Display','off');
        gamma = 1e4;
        
        %Shift to single integrator
        xi = x(1:2, :) + lambda*[cos(x(3, :)) ; sin(x(3, :))];
        
        dxi = uni2int(dxu, x, lambda);

        %Determine dimension of constraints
        count = 0; 
        for i = 1:(N-1)
            for j = (i+1):N
                count = count + 1;
            end
        end    
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        A=zeros(count, 2*N);  b=zeros(count, 1);
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(xi(1:2,i)-xi(1:2,j))^2-(safetyRadius + 2*lambda)^2;
                Anew = zeros(1,2*N);
                Anew((2*i-1):(2*i)) = - 2*(xi(:,i)-xi(:,j))';
                Anew((2*j-1):(2*j)) =  2*(xi(:,i)-xi(:,j))';
                A((i * (N-1))+j, :) = Anew;
                b((i * (N-1))+j, :) = gamma*h^3;
            end
        end

        %Solve QP program generated earlier
        vhat = reshape(dxi,2*N,1);
        k_relax = 1;
        H = 2*eye(2*N);
        f = -2*vhat;
        relaxationVariables = k_relax*1*ones(1, 2*N);
        vnew = quadprog(H, double(f), A, double(b), [],[], -relaxationVariables, relaxationVariables,[],opts);

        %Set robot velocities to new velocities
        dx = int2uni(reshape(vnew, 2, N), x, lambda);

end

