%%% Script for breaking large Jacobian into smaller matrices,
%%% solving the qr of the smaller system.  Then combine and solve once, 
%%% more to solve the original system.

close all; clear; clc;

%% Add path to utils.
addpath('~/MATLAB/utils/ba_utils/');
addpath('~/Downloads/SuiteSparse/SPQR/MATLAB');

%% Parameters.
is_file_transposed = true;
test_jacobian = 'test1_734_10053';
J_binary_file = ['~/for_matt/sparse_matrices/J_t_' test_jacobian '.dat'];  % Original matrix.

%% Parse data.
J_sparse = ReadSparseMatrix(J_binary_file);
if is_file_transposed 
    J_sparse = J_sparse';
end
[m, n] = size(J_sparse);
disp(['J nnz: ' num2str(nnz(J_sparse)) ' (' ...
            num2str(100 * nnz(J_sparse) / (m * n), '%04f') '%)']);
%% Assumed rhs.
b = ones(size(J_sparse, 1), 1);

%% Matlab Residual.
X_matlab = J_sparse \ b;
res_matlab = J_sparse * X_matlab - b;

%% SuiteSparse for matlab.
tic;
[C_qr, R_qr, P_qr] = qr(J_sparse, b);
J_time = toc;
disp(['SPQR time taken to solve J: ' num2str(J_time)]);

x_ssm = R_qr \ C_qr;
x_ssm = P_qr * x_ssm;
res_ssm = J_sparse * x_ssm - b;

%% Total residual difference.
err_matlab = norm(res_matlab);
err_ssm = norm(res_ssm);

disp(['Relative error over J (matlab vs SuiteSparse): ' ...
      num2str(100 * abs(err_matlab - err_ssm) / err_matlab, ...
              '%0.4f') ' %']);
       
%% Solve by subsystems.
[m, n] = size(J_sparse);
for num_splits = 3:floor(m / n)
    x = subsolve_qr(J_sparse, num_splits, b);
    res_sub = J_sparse * x - b;
    err_sub = norm(res_sub);
    disp(['Relative error (original vs subsytem): ' ...
        num2str(100 * abs(err_matlab - err_sub) / err_matlab, ...
                  '%0.4e') ' %']);
end
%% Helper functions.
       
function [x] = subsolve_qr(J, num_sub_matrices, b)
    % TODO(mpoutler) R becomes singular.  Something is not correct with
    % Building R_huge -- most likely to do with the permutation.
    
    % Solve for Jx = b.
    [m, n] = size(J);
    block_size = floor(m / num_sub_matrices);
    % TODO(mpoulter) use sparse arrays and build sparse matrix at the end.
    %% Built the large system of solved subsystems.
    R_huge = sparse(m, n);
    y_huge = zeros(m, 1);
    test_indices = zeros(size(y_huge));
    last_filled = 0;
    for i = 1 : num_sub_matrices - 1
        start_row = block_size * (i-1) + 1;
        end_row= start_row + block_size - 1;
        
        A = J(start_row:end_row, :);
        tic;
        [c, R, P] = qr(A, b(start_row:end_row, :));
        time = toc;

        disp([num2str(i) ' sub-matrix time: ' num2str(time)]);
        disp([num2str(i) ' nnz: ' num2str(nnz(R)) ' (' ...
            num2str(100 * nnz(R) / (m * n), '%04f') '%)']);
    
        y_huge(start_row:end_row) = c;
        R_huge(start_row:end_row, :) = R * P';
        
        %%% For testing.
        test_indices(start_row:end_row) = start_row:end_row;
        last_filled = end_row;
    end
    
    %  Final block to adjust for odd cases.
    start_row = last_filled + 1;
    A = J(start_row:end, :); 
    tic;
    [c, R, P] = qr(A, b(start_row:end, :));
    time = toc;
    disp([num2str(i+1) ' sub-matrix time: ' num2str(time)]);
        
    y_huge(start_row:end, :) = c;
    R_huge(start_row:end, :) = R * P';
    
    test_indices(start_row:end) = start_row:(size(y_huge, 1));
    assert(all(diff(test_indices) == 1));
    assert(sum(test_indices) == ...
        ((length(test_indices)^2 + length(test_indices))/2));  % n(n+1)/2
          
    tic; 
    [c, R, P] = qr(R_huge, y_huge);
    x = P * (R \ c);
    time = toc;
    [m, n] = size(R);
    disp(['Reconstructed size: ' num2str(m) ' x ' num2str(n)]);
    disp(['Reconstructed nnz: ' num2str(nnz(R)) ' (' ...
        num2str(100 * nnz(R) / (m * n), '%04f') '%)']);
    disp(['Reconstructed time: ' num2str(time)]);
    
    
    
    
end


