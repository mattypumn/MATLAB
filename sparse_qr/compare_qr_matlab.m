close all; clear; clc;

%% Add path to utils.
addpath('~/redwood_ws/MATLAB/utils/ba_utils/');
addpath('~/Downloads/SuiteSparse/SPQR/MATLAB');

%% Parameters.
is_J_transposed = true;
test_jacobian = 'test1_734_10053';
J_binary_file = ['~/for_matt/sparse_matrices/J_t_' test_jacobian '.dat'];  % Original matrix.

%% Parse data.
J_sparse = ReadSparseMatrix(J_binary_file);
if is_J_transposed 
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
              '%0.4f') ' %']);
       
%% Solve by subsystems.
x = subsolve_qr(J_sparse, 3, b);

res_sub = J_sparse * x - b;
err_sub = norm(res_sub);
disp(['Relative error (original vs subsytem): ' ...
      num2str(100 * abs(err_matlab - err_sub) / err_matlab, ...
              '%0.4f') ' %']);



%% Helper functions.
       
function x = subsolve_qr(J, num_sub_matrices, b) 
    % TODO(mpoutler) R becomes singular.  Something is not correct with
    % Building R_huge -- most likely to do with the permutation.
    
    % Solve for Jx = b.
    [m, n] = size(J);
    block_size = floor(m / num_sub_matrices);
    m_new = n * num_sub_matrices;
    % TODO(mpoulter) use sparse arrays and build sparse matrix at the end.
    %% Built the large system of solved subsystems.
    R_huge = sparse(m_new, n);
    y = zeros(m_new, 1);
    for i = 1 : num_sub_matrices - 1
        start_row = n * (i-1) + 1;
        A = J(start_row:start_row + block_size - 1, :); 
        tic;
        [C, R, p] = spqr(A, b(start_row:start_row + block_size - 1, :), 0);
        P = speye(size(R));
        P = P(:,p);
        time = toc;
        disp(['sub-matrix time: ' num2str(time)]);
        y(start_row:start_row + n - 1, :) = C(:);
        R_huge(start_row:start_row + n - 1, :) = R * P';
    end
    %  Final block to adjust for odd cases.
    i = i + 1;
    start_row = n * (i-1) + 1;
    A = J(start_row:start_row + block_size - 1, :); 
    tic;
    [C, R, p] = spqr(A, b(start_row:start_row + block_size - 1, :), 0);
    time = toc;
    P = speye(size(R));
    P = P(:,p);
    disp(['sub-matrix time: ' num2str(time)]);
    %%  There are some errors here with the permutation matrix.
%     y(start_row:start_row + n - 1, :) = P * C(:);
%     R_huge(start_row:start_row + n - 1, :) = P * R;
    y(start_row:start_row + n - 1, :) = P * C(:);
    R_huge(start_row:start_row + n - 1, :) = R * P;
    
    tic; 
    [C, R, p] = spqr(R_huge, y, 0);
    x = R \ C;
    x(p) = x;
    time = toc;
    disp(['Reconstructed time: ' num2str(time)]);
end

