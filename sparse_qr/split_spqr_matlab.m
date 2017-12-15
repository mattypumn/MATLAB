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
% x = subsolve_spqr(J_sparse, 3, b);
[m, n] = size(J_sparse);
data = {};
for iter = 1 : 10
    dat_i = 1;
    for num_splits = 3:floor(m / n)
        [x, dat] = subsolve_thin_spqr(J_sparse, num_splits, b);
        data{iter, dat_i} = dat;
        dat_i = dat_i + 1;
        res_sub = J_sparse * x - b;
        err_sub = norm(res_sub);
        disp(['Relative error (original vs subsytem): ' ...
            num2str(100 * abs(err_matlab - err_sub) / err_matlab, ...
                      '%0.4e') ' %']);
    end
end

disp('Finished.');


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



function [x, data] = subsolve_thin_spqr(J, num_sub_matrices, b)
    % TODO(mpoutler) R becomes singular.  Something is not correct with
    % Building R_huge -- most likely to do with the permutation.
    data.sub_time = [];
    data.full_time = [];
    data.sub_J_nnz = [];
    data.sub_R_nnz = [];
    data.full_R_huge_nnz = [];
    data.R_huge_rows = [];
    data.R_final_nnz = [];

    
    % Solve for Jx = b.
    [m, n] = size(J);
    block_size = floor(m / num_sub_matrices);
    % TODO(mpoulter) use sparse arrays and build sparse matrix at the end.
    %% Build the large system of solved subsystems.
    R_huge = sparse(n * num_sub_matrices, n);       % shell
    y_huge = zeros(n * num_sub_matrices, 1);
    test_indices = zeros(n * num_sub_matrices, 1);
    last_extracted = 0;

    %% For each sub matrix.
    for i = 1 : num_sub_matrices - 1
        %% Extract.
        start_extract_row = block_size * (i-1) + 1;
        end_extract_row = start_extract_row + block_size - 1;
        start_insert_row = n * (i-1) + 1;
        end_insert_row = start_insert_row + n - 1;
               
        A = J(start_extract_row:end_extract_row, :);
        
        %% Time.
        tic;
        [C, R, p] = spqr(A, b(start_extract_row:end_extract_row, :), 0);
        time = toc;
        
        %% Display and save.
        disp([num2str(i) ' sub-matrix time: ' num2str(time)]);
        data.sub_time = [data.sub_time, time];
        data.sub_R_nnz = [data.sub_R_nnz, nnz(R)];
        data.sub_J_nnz = [data.sub_J_nnz, nnz(A)];
                
        %% Permute our R matrix so all match up.
        P = speye(size(R, 2));
        P = P(:, p);
        y_huge(start_insert_row:end_insert_row) = C;
        R_huge(start_insert_row:end_insert_row, :) = R * P';
        
        %% For testing.  Later this is used to assert indexing.
        test_indices(start_insert_row:end_insert_row) = ...
            start_insert_row:end_insert_row;
        last_extracted = end_extract_row;
    end
    
    %%  Final block for uneven split over num_sub_matrices.
    % Extract.
    start_extract_row = last_extracted + 1;
    i = i + 1;
    start_insert_row = n * (i-1) + 1;
    A = J(start_extract_row:end, :); 
    
    %% Time.
    tic;
    [C, R, p] = spqr(A, b(start_extract_row:end, :), 0);
    time = toc;
    
    %% Display and save (Final sub-matrix).
    disp([num2str(i) ' sub-matrix time: ' num2str(time)]);
    data.sub_time = [data.sub_time, time];
    data.sub_R_nnz = [data.sub_R_nnz, nnz(R)];
    data.sub_J_nnz = [data.sub_J_nnz, nnz(A)];
    
    %% Insert R into R_huge.
    P = speye(size(R,2));
    P = P(:,p);
    y_huge(start_insert_row:end, :) = C;
    R_huge(start_insert_row:end, :) = R * P';
    
    %% Assert proper indexing.
    test_indices(start_insert_row:end) = start_insert_row:(size(test_indices, 1));
    assert(all(diff(test_indices) == 1));
    assert(sum(test_indices) == ...
        ((length(test_indices)^2 + length(test_indices))/2));  % n(n+1)/2
      
    %% Time R_huge decomposition.
    tic; 
    [C, R, p] = spqr(R_huge, y_huge, 0);
    time = toc;
    
    %% Display Save data.
    [m, ~] = size(R);
    disp(['Reconstructed matrix time: ' num2str(time)]);
    data.full_time = time;
    data.full_R_huge_nnz = nnz(R_huge);
    data.R_huge_rows = m;
    data.R_final_nnz = nnz(R);

    %% Final solve.  'x' is returned.
    P = speye(size(R,2));
    P = P(:,p);
    x = P * (R \ C);
end
