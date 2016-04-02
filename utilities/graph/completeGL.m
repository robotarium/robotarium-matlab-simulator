function [ L ] = completeGL( n )
%COMPLETEGL Summary of this function goes here
%   Detailed explanation goes here

    L = ones(n,n) - n * eye(n);
end

