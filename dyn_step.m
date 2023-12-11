function [x_next] = dyn_step(x_in,u,dt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x_next = x_in + u .* dt;

end