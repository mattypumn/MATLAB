%%%  Note. Parameters and Setup are machine / data specific and may require
%%%        manipulations to run.

close all; clear; clc;

%% Parameters.
mat_name = '33R_600cols';
data_geration_dir = '~/RunSuiteSparse/data/';
kTransposeOriginalMatrix = false;
kTransposeR = false;

%% Setup
original_matrix_file = ['/media/mattp/DATA2/SparseMatrices/' mat_name '.dat'];  % Original matrix.
rhs_matrix_file = ['~/RunSuiteSparse/data/' mat_name '_random_rhs.txt'];

float_R_file = [data_geration_dir mat_name '_R_' 'float' '.dat'];
float_QtB_file = [data_geration_dir mat_name '_QtB_' 'float' '.txt'];
float_permutation_file = [data_geration_dir mat_name '_perm_' 'float' '.txt'];

double_R_file = [data_geration_dir mat_name '_R_' 'double' '.dat'];
double_QtB_file = [data_geration_dir mat_name '_QtB_' 'double' '.txt'];
double_permutation_file = [data_geration_dir mat_name '_perm_' 'double' '.txt'];

%% Add path to utils.
addpath('~/MATLAB/utils/ba_utils/');
addpath('~/Libraries/SuiteSparseDouble/SPQR/MATLAB');

%% Load Data.
J_sparse = ReadSparseMatrix(original_matrix_file);
if kTransposeOriginalMatrix 
    J_sparse = J_sparse';
end
[m, n] = size(J_sparse);
disp(['J size: ' num2str(m) ' x ' num2str(n)]);
disp(['J nnz: ' num2str(nnz(J_sparse)) ' (' ...
            num2str(100 * nnz(J_sparse) / (m * n), '%04f') '%)']);

RHS_matrix = dlmread(rhs_matrix_file);

QtB_double = dlmread(double_QtB_file);
R_double = ReadSparseMatrix(double_R_file);
p_double = dlmread(double_permutation_file); p_double = p_double + 1; % 1-indexing.

QtB_float = dlmread(float_QtB_file);
R_float = ReadSparseMatrix(float_R_file);
p_float = dlmread(float_permutation_file); p_float = p_float + 1;

% Qtb_float
% QtB_float = dlmread(float_QtB_file);

%% Solve with SPQR.
opts = struct();
opts.ordering = 'fixed';
opts.econ = n;
[QtB_ss, R_ss, P_ss] = spqr(J_sparse, RHS_matrix, opts);

%% Calculate residuals.
X_ss = R_ss \ QtB_ss;
X_double = R_double \ QtB_double; X_double(p_double, :) = X_double;
X_float = R_float \ QtB_float; X_float(p_float, :) = X_float;


%%  Compare R matrices.

%% Compare R matrices.
res_R_matlabVcpp = norm(abs(R_ss) - abs(R_double), 'fro');
res_R_doubleVfloat = norm(abs(R_double) - abs(R_float), 'fro');

disp(['|| abs(R_matlab) - abs(R_double) ||   = ' num2str(res_R_matlabVcpp)]);
disp(['|| abs(R_double) - abs(R_float) ||   = ' num2str(res_R_doubleVfloat)]);

%% Compare QtB.
res_Qtb_matlabVdouble = norm(abs(QtB_ss) - abs(QtB_double), 'fro');
res_Qtb_doubleVfloat = norm(abs(QtB_double) - abs(QtB_float), 'fro');

disp(['|| abs(QtB_matlab) - abs(QtB_double) ||   = ' num2str(res_Qtb_matlabVdouble)]);
disp(['|| abs(QtB_double) - abs(QtB_float) ||   = ' num2str(res_Qtb_doubleVfloat)]);

%% Compare residuals.
res_ss = norm(RHS_matrix - J_sparse * X_ss, 'fro');
res_double = norm(RHS_matrix - J_sparse * X_double, 'fro');
res_float = norm(RHS_matrix - J_sparse * X_float, 'fro');

disp(['|| B - J * X_matlab ||   = ' num2str(res_ss, '%.10f')]);
disp(['|| B - J * X_double ||   = ' num2str(res_double, '%.10f')]);
disp(['|| B - J * X_float ||   = ' num2str(res_float, '%.10f')]);

%% Compare A' * A - R' * R.
matlab_sys_norm = norm(J_sparse' * J_sparse - R_ss' * R_ss, 'fro');
double_sys_norm = norm(J_sparse' * J_sparse - R_double' * R_double, 'fro');
float_sys_norm = norm(J_sparse' * J_sparse - R_float' * R_float, 'fro');


disp(['|| J'' * J - R_matlab'' * R_matlab ||   = ' num2str(matlab_sys_norm)]);
disp(['|| J'' * J - R_double'' * R_double ||   = ' num2str(double_sys_norm)]);
disp(['|| J'' * J - R_float'' * R_float ||   = ' num2str(float_sys_norm)]);
