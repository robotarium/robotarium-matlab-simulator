function [ L ] = lineGL(n)
%LINEGRAPH Summary of this function goes here
%   Detailed explanation goes here
    L = 2*eye(n) - diag(ones(1,(n-1)), 1) - diag(ones(1, n-1), -1);
    L(1, 1) = 1; 
    L(n, n) = 1;
end

