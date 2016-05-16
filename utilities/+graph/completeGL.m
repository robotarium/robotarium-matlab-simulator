function [ L ] = completeGL( n )
%COMPLETEGL Summary of this function goes here
%   Detailed explanation goes here

    L = n * eye(n) - ones(n,n);
end

