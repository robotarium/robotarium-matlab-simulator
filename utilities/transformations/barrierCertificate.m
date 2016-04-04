function [ dx ] = barrierCertificate(dxi, x, safetyRadius)
%BARRIERCERTIFICATE Summary of this function goes here
%   Detailed explanation goes here

            % barrier certificates, centralized case
        N = size(dxi, 2);
        opts = optimoptions('quadprog','Display','off');
        gamma = 1e4;
        x = x(1:2, :);

        % apply barrier certificates
        count = 0; 
        for i = 1:(N-1)
            for j = (i+1):N
                count = count + 1;
            end
        end    
        
        A=zeros(count, 2*N);  b=zeros(count, 1);
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(x(1:2,i)-x(1:2,j))^2-safetyRadius^2;
                Anew = zeros(1,2*N);
                Anew((2*i-1):(2*i)) = - 2*(x(:,i)-x(:,j))';
                Anew((2*j-1):(2*j)) =  2*(x(:,i)-x(:,j))';
                A((i * (N-1))+j, :) = Anew;
                b((i * (N-1))+j, :) = gamma*h^3;
            end
        end

        vhat = reshape(dxi,2*N,1);
        k_relax = 1;
        H = 2*eye(2*N);
        f = -2*vhat;
        relaxationVariables = k_relax*0.1*ones(1, 2*N);
        vnew = quadprog(H, double(f), A, double(b), [],[], -relaxationVariables, relaxationVariables,[],opts);

        %Set robot velocities to new velocities
        dx = reshape(vnew, 2, N);

end

