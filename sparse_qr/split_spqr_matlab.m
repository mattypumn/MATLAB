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

%% Assumed rhs.
b = ones(size(J_sparse, 1), 1);

%% Matlab Residual.
X_matlab = J_sparse \ b;
res_matlab = J_sparse * X_matlab - b;

%% SuiteSparse for matlab.
tic;
[C_ssm, R_ssm, p_ssm] = spqr(J_sparse, b, 0);
J_time = toc;
disp(['SPQR time taken to solve J: ' num2str(J_time)]);

x_ssm = R_ssm \ C_ssm;
x_ssm(p_ssm) = x_ssm;
res_ssm = J_sparse * x_ssm - b;

%% Total residual difference.
err_matlab = norm(res_matlab);
err_ssm = norm(res_ssm);

disp(['Relative error over J (matlab vs SuiteSparse): ' ...
      num2str(100 * abs(err_matlab - err_ssm) / err_matlab, ...
              '%0.4e') ' %']);
       
%% Solve by subsystems.
x = subsolve_spqr(J_sparse, 3, b);
x2 = subsolve_thin_spqr(J_sparse, 3, b);

res_sub = J_sparse * x - b;
err_sub = norm(res_sub);
disp(['Relative error (original vs subsytem): ' ...
      num2str(100 * abs(err_matlab - err_sub) / err_matlab, ...
              '%0.4e') ' %']);

res_sub_thin = J_sparse * x2 - b;
err_sub_thin = norm(res_sub_thin);
disp(['Relative error (original vs subsystem-thin): ' ...
      num2str(100 * abs(err_matlab - err_sub_thin) / err_matlab, ...
              '%0.4e') ' %']);


%% Helper functions.
       
function x = subsolve_spqr(J, num_sub_matrices, b)
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
        [C, R, P] = spqr(A, b(start_row:end_row, :));
        time = toc;

        disp([num2str(i) ' sub-matrix time: ' num2str(time)]);
        y_huge(start_row:end_row) = C;
        R_huge(start_row:end_row, :) = R * P';
        
        %%% For testing.
        test_indices(start_row:end_row) = start_row:end_row;
        last_filled = end_row;
    end
    
    %  Final block to adjust for odd cases.
    start_row = last_filled + 1;
    A = J(start_row:end, :); 
    tic;
    [C, R, P] = spqr(A, b(start_row:end, :));
    time = toc;
    disp(['final sub-matrix time: ' num2str(time)]);
        
    y_huge(start_row:end, :) = C;
    R_huge(start_row:end, :) = R * P';
    
    test_indices(start_row:end) = start_row:(size(y_huge, 1));
    assert(all(diff(test_indices) == 1));
    assert(sum(test_indices) == ...
        ((length(test_indices)^2 + length(test_indices))/2));  % n(n+1)/2
      
    
    tic; 
    [C, R, P] = spqr(R_huge, y_huge);
    time = toc;
    x = P * (R \ C);
    disp(['Reconstructed matrix time: ' num2str(time)]);
end



function x = subsolve_thin_spqr(J, num_sub_matrices, b)
    % TODO(mpoutler) R becomes singular.  Something is not correct with
    % Building R_huge -- most likely to do with the permutation.
    
    % Solve for Jx = b.
    [m, n] = size(J);
    block_size = floor(m / num_sub_matrices);
    % TODO(mpoulter) use sparse arrays and build sparse matrix at the end.
    %% Built the large system of solved subsystems.
    R_huge = sparse(n * num_sub_matrices, n);
    y_huge = zeros(n * num_sub_matrices, 1);
    test_indices = zeros(n, 1);
    last_filled = 0;
    for i = 1 : num_sub_matrices - 1
        start_extract_row = block_size * (i-1) + 1;
        end_extract_row = start_extract_row + block_size - 1;
        
        start_insert_row = n * (i-1) + 1;
        end_insert_row = start_insert_row + n - 1;
                
        A = J(start_extract_row:end_extract_row, :);
        tic;
        [C, R, p] = spqr(A, b(start_extract_row:end_extract_row, :), 0);
        time = toc;
        disp([num2str(i) ' sub-matrix time: ' num2str(time)]);
        
        P = speye(size(R, 2));
        P = P(:, p);
        y_huge(start_insert_row:end_insert_row) = C;
        R_huge(start_insert_row:end_insert_row, :) = R * P';
        
        %%% For testing.
        test_indices(start_insert_row:end_insert_row) = ...
            start_insert_row:end_insert_row;
        last_filled = end_extract_row;
    end
    
    %  Final block to adjust for odd cases.
    start_extract_row = last_filled + 1;
    i = i + 1;
    start_insert_row = n * (i-1) + 1;
    
    A = J(start_extract_row:end, :); 
    tic;
    [C, R, p] = spqr(A, b(start_extract_row:end, :), 0);
    time = toc;
    disp(['final sub-matrix time: ' num2str(time)]);
    
    P = speye(size(R,2));
    P = P(:,p);
    y_huge(start_insert_row:end, :) = C;
    R_huge(start_insert_row:end, :) = R * P';
    
    test_indices(start_insert_row:end) = start_extract_row:(size(y_huge, 1));
    assert(all(diff(test_indices) == 1));
    assert(sum(test_indices) == ...
        ((length(test_indices)^2 + length(test_indices))/2));  % n(n+1)/2
      
    
    tic; 
    [C, R, p] = spqr(R_huge, y_huge, 0);
    time = toc;

    P = speye(size(R,2));
    P = P(:,p);
    x = P * (R \ C);
    disp(['Reconstructed matrix time: ' num2str(time)]);
end
