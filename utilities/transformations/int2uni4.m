function [ dxu ] = int2uni4(dxq, x, q, lambda)
%INT2UNI4 Summary of this function goes here
%   Detailed explanation goes here

        [~, N] = size(dxq);
        opts = optimoptions('quadprog','Display','off');
        
        xi = x(1:2, :) + [cos(x(3, :)) ; sin(x(3, :))]*(lambda/2);
        
        A = []; 
        b = []; 
        h = @(xi, q) (lambda/2)^2 - norm(xi - q)^2;
        gamma = 1; 
        
        for i = 1:N
            
            xiq = (xi(:, i) - q(:, i))';            
            
            A_row = zeros(1, 2*N);
            A_row((2*i-1):(2*i)) = -xiq; 
            b_row = -gamma*h(xi(:, i), q(:, i)) - xiq*dxq(:, i);
            A = [A ; A_row];
            b = [b ; b_row];
        end        
        
        A = -A;
        b = -b;

%         A = []; 
%         b = [];
%         h = @(x, q) lambda - norm(x - q);
%         g_x = @(x) [cos(x(3)) 0 ; sin(x(3)) 0];
%         LgPhi = @(x) [cos(x(3)) -lambda*sin(x(3)); sin(x(3)) lambda*cos(x(3))];
%         gamma = 1e-4;
%         
%         for i = 1:N
%             
%             xq = (x(1:2, i) - q(:, i))' / norm(x(1:2, i) - q(:, i));
%             
%             % Linear velocity constraint
%             A_row = zeros(1, 2*N);
%             A_row((2*i - 1):(2*i)) = -xq*g_x(x); 
%             b_row = -gamma*h(x(1:2, i), q(:, i)) - xq*dxq(:, i);
%             A = [A ; A_row]; 
%             b = [b ; b_row];
%             xq
%             %Angular velocity constraint 
%             A_row = zeros(1, 2*N);
%             A_row((2*i - 1):(2*i)) = -xq*(g_x(x) - LgPhi(x));
%             (g_x(x) - LgPhi(x))
%             b_row = -gamma*h(x(1:2, i), q(:, i));
%             A = [A ; A_row]; 
%             b = [b ; b_row];            
%         end
%         
%         A = -A
%         b = -b
        
        %A 
        %b

        H = 2*eye(2*N);   
        
        %If we can't solve the current problem, relax the conditions
        dxu = int2uni(reshape(quadprog(H, [], A, b, [],[], [], [], [], opts), 2, N), x, lambda);
end

