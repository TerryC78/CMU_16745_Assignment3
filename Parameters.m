function [ plan ] = Parameters( fname )
%Parameters Retrieves footstep plan parameters from input filename.
%   fname: input filename
%   plan: struct containing plan parameters

    tmp = load(fname);
    plan.time = tmp(:, 1);
    plan.p_x = tmp(:, 2);
    plan.p_y = tmp(:, 3);
    plan.stance_type = tmp(:, 4);
end
