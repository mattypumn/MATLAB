clc;
clear;

J = ReadSparseMatrix('~/J_t.dat')';
N = size(J, 2); % Number of cols
M = size(J, 1); % Number of rows
residual = ones(M, 1);

dx_benchmark = J \ residual;

SEP=10;

indices = round(linspace(0, M, SEP));
% Check for uniqueness
assert(length(indices) == length(unique(indices)));
assert(length(indices) > 2);

partial_Rs = cell(length(indices) - 1, 1);
partial_rhs = cell(length(indices) - 1, 1);

total_new_R = [];
total_rhs = [];

for i = 2:length(indices)
    start_index = indices(i - 1) + 1;
    end_index = indices(i);
    
    J_partial = J(start_index:end_index, :);
    partial_residual = residual(start_index:end_index, :);
    
    % J * E = Q * R ==> J = Q * R * E^T
    [Q, R, E] = qr(J_partial, 'matrix');
    partial_rhs{i - 1} = Q' * partial_residual;
    partial_Rs{i - 1} = R * E';
    
    total_new_R = [total_new_R; partial_Rs{i - 1}];
    total_rhs = [total_rhs; partial_rhs{i - 1}];
end

dx = total_new_R \ total_rhs;
accuracy = norm(dx - dx_benchmark)