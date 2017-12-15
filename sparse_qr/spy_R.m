close all; clear; clc;

%% Parameters.
%%% Information for Jacobian.
is_jac_transposed = true;
test_jacobian = 'test1_734_10053';
J_binary_file = ['~/for_matt/sparse_matrices/J_t_' test_jacobian '.dat'];  % Original matrix.

%%% Information for [~, R] = qr(J);
R_file = '/usr/local/google/home/mpoulter/RunSuiteSparse/data/R_0.mat';
is_R_transposed = false;

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

R_c_impl = ReadSparseMatrix(R_file);
if is_R_transposed 
    R_c_impl = R_c_impl';
end
disp(['Loaded: ' R_file]);
[m_c, n_c] = size(R_c_impl);

%%  Use matlab's QR to get the R for the Jacobian.
b = ones([size(J_sparse, 1), 1]);
[C, R_qr, p] = qr(J_sparse, b, 0);
[m_qr, n_qr] = size(R_qr);

%% Use SPQR to get the R for the Jacobian.
[C, R_spqr, p] = spqr(J_sparse, b, 0);
[m_spqr, n_spqr] = size(R_spqr);

%% Report results
figure();
spy(R_qr);
hold on;
title(['Matlab QR (' num2str(m_qr) 'x' num2str(n_qr) ']']);

figure();
spy(R_spqr);
hold on;
title(['Matlab SPQR (' num2str(m_spqr) 'x' num2str(n_spqr) ']']);

figure();
spy(R_c_impl);
hold on;
title(['C-implementation R (' num2str(m_c) 'x' num2str(n_c) ']']);
