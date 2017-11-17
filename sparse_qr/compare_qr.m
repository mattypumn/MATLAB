close all; clear; clc;

%% Add path to utils.
addpath('~/redwood_ws/MATLAB/utils/ba_utils/');
addpath('~/Downloads/SuiteSparse/SPQR/MATLAB');

%% Parameters.
is_J_transposed = true;
% JE = QR
test_jacobian = 'test1_734_10053';
J_binary_file = ['~/for_matt/sparse_matrices/J_t_' test_jacobian '.dat'];  % Original matrix.

% Most recently ran.
R_binary_file = '~/RunSuiteSparse/data/R_0.mat';  % Decomposed R.
Qt_b_ascii_file = '~/RunSuiteSparse/data/QT_b_0.mat'; % Q'*b for QRE'x=b.
Perm_ascii_file = '~/RunSuiteSparse/data/Permutation_0.mat';
x_ascii_file = '~/RunSuiteSparse/data/x_solved_0.mat';

% Proper files.
% R_binary_file = ['~/RunSuiteSparse/data/R_' test_jacobian '.mat'];  % Decomposed R.
% Qt_b_ascii_file = ['~/RunSuiteSparse/data/QT_b_' test_jacobian '.mat']; % Q'*b for QRE'x=b.
% Perm_ascii_file = ['~/RunSuiteSparse/data/Permutation_' test_jacobian '.mat'];

%% Parse data.
QT_b_suitesparse = dlmread(Qt_b_ascii_file);
R_suitesparse = ReadSparseMatrix(R_binary_file);
J_sparse = ReadSparseMatrix(J_binary_file);
if is_J_transposed 
    J_sparse = J_sparse';
end


e_suitesparse = dlmread(Perm_ascii_file) + 1;  % Adjusting for the 0-indexing of C++

x_suitesparse = dlmread(x_ascii_file);

%% Build permutation matrix, E, s.t. J*E = Q*R
E_suitesparse = speye(size(R_suitesparse));
E_suitesparse = E_suitesparse(:, e_suitesparse);

%% Assumed rhs.
b = ones(size(J_sparse, 1), 1);

%% Matlab Residual.
X_matlab = J_sparse \ b;
res_matlab = J_sparse * X_matlab - b;
% 

%% SuitesSparse Residual.
% X_suitesparse = E_suitesparse * (R_suitesparse \ QT_b_suitesparse);
% res_suitesparse = J_sparse * X_suitesparse - b;


%% Suitesparse for matlab!
% [C_ssm, R_ssm] = spqr(J_sparse, b);
[C_ssm2, R_ssm2, P] = spqr(J_sparse, b);
[C_ssm3, R_ssm3, p] = spqr(J_sparse, b, 0);

x2 = P * (R_ssm2 \ C_ssm2);
x3 = R_ssm3 \ C_ssm3;
x3(p) = x3;

build_P = speye(size(P));  
build_P = build_P(:,p);   % Proper way to build a Permutation matrix from p-vector
diff_P_build = sum(sum(abs(P - build_P)));

diff_x_ssm = sum(sum(x3 - x2));
diff_x_ssm2suitesparse = sum(sum(abs(x3 - x_suitesparse)));
diff_C = sum(sum(abs(QT_b_suitesparse - C_ssm3)));
diff_R = sum(sum(abs(R_suitesparse - R_ssm3)));
diff_p = sum(sum(abs(p' - e_suitesparse)));
res_ssm = J_sparse * x2 - b;

%% Compare errors.
err_matlab = norm(res_matlab);
err_suitesparse = norm(res_suitesparse);

disp(['Relative error spqr C++: ' ...
      num2str(100 * abs(err_matlab - err_suitesparse) / err_matlab, ...
              '%0.4f') '%']);
disp(['Relative error spqr MATLAB: ' ...
      num2str(100 * abs(err_matlab - err_ssm) / err_matlab, ...
              '%0.4f') ' %']);
