function [ plan ] = Parameters( fname )
%Parameters Summary of this function goes here
%   Detailed explanation goes here

    tmp = load(fname);
    plan.time = tmp(:, 1);
    plan.p_x = tmp(:, 2);
    plan.p_y = tmp(:, 3);
    plan.stance_type = tmp(:, 4);
end
