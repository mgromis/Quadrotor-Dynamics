function [ args ] = getTraj( t1, t2, p1, p2, v1, v2, a1, a2 )
%GETTRAJ Summary of this function goes here
%   Detailed explanation goes here

args = zeros(length(p1), 6);
A = [1 t1 t1^2 t1^3 t1^4 t1^5;
    0 1 2*t1 3*t1^2 4*t1^3 5*t1^4;
    0 0 2 6*t1 12*t1^2 20*t1^3;
    1 t2 t2^2 t2^3 t2^4 t2^5;
    0 1 2*t2 3*t2^2 4*t2^3 5*t2^4;
    0 0 2 6*t2 12*t2^2 20*t2^3];
    
    
for k = 1:length(p1)
    c = [p1(k), v1(k), a1(k), p2(k), v2(k), a2(k)]';
    args(k,:) = A\c;

end

