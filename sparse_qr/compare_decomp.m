close all; clear; clc;

%% Parameters.
%%% Information for Jacobian.
is_jac_transposed = true;
test_jacobian = 'test1_734_10053';
J_binary_file = ['~/for_matt/sparse_matrices/J_t_' test_jacobian '.dat'];  % Original matrix.

%%% Information for [~, R] = qr(J);
R_file = '/usr/local/google/home/mpoulter/RunSuiteSparse/data/R_0.mat';
is_R_transposed = false;

Qt_b_file = '/usr/local/google/home/mpoulter/RunSuiteSparse/data/QT_b_0.mat';
p_file = '/usr/local/google/home/mpoulter/RunSuiteSparse/data/Permutation_0.mat';

%% Add path to utils.
addpath('~/MATLAB/utils/ba_utils/');
addpath('~/Downloads/SuiteSparse/SPQR/MATLAB');

%% Parse data.
J_sparse = ReadSparseMatrix(J_binary_file);
if is_jac_transposed 
    J_sparse = J_sparse';
end
disp(['Loaded: ' J_binary_file]);
disp(['Jacobian nnz: ' num2str(nnz(J_sparse))]);

%%%  C  R-decomp
R_c_impl = ReadSparseMatrix(R_file);
if is_R_transposed 
    R_c_impl = R_c_impl';
end
disp(['Loaded: ' R_file]);
[m_c, n_c] = size(R_c_impl);

%%%  Q'*b
Qt_b_c_impl = dlmread(Qt_b_file);

%%%  permutation vector.
p_c_impl = dlmread(p_file);

%%  Assumed Right hand side of system.
b = ones(size(J_sparse,1), 1);

%% Calculate with c-implementation.
p_c_impl = p_c_impl + 1;   %% Fix 0-indexing from c.
P_c_impl = speye(length(p_c_impl));
P_c_impl = P_c_impl(:,p_c_impl);
x_c_impl = P_c_impl * (R_c_impl \ Qt_b_c_impl);

%% Calculate with matlab QR.
[C_qr, R_qr, p_qr] = qr(J_sparse, b, 0);
P_qr = speye(length(p_qr));
P_qr = P_qr(:, p_qr);
x_qr = P_qr * (R_qr \ C_qr);

%% Calculate with matlab SPQR.
[C_spqr, R_spqr, p_spqr] = spqr(J_sparse, b, 0);
P_spqr = speye(length(p_spqr));
P_spqr = P_spqr(:, p_spqr);
x_spqr = P_spqr * (R_spqr \ C_spqr);

%% Compare.
res_c_imp = J_sparse * x_c_impl - b;
res_qr = J_sparse * x_qr - b;
res_spqr = J_sparse * x_spqr - b;

disp(['Res-norm qr: ' num2str(norm(res_qr))]);
disp(['Res-norm matlab-spqr: ' num2str(norm(res_spqr))]);
disp(['Res-norm c-spqr: ' num2str(norm(res_c_imp))]);









