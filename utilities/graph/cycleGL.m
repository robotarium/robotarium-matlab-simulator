function [ L ] = cycleGL( n )
%C_graph Generates a graph Laplacian for a cycle graph.
%  The order is assumed to be 1->2->3->...->n
    L = 2*eye(n) - diag(ones(1,(n-1)), 1) - diag(ones(1,(n-1)), -1);
    L(n, 1) = -1;
    L(1, n) = -1;
end

