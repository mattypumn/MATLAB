%%%  Note. Parameters and Setup are machine / data specific and may require
%%%        manipulations to run.

%% Parameters.
mat_name = 'J_t_d8_437_195';
data_geration_dir = '~/RunSuiteSparse/data/';
kTransposeOriginalMatrix = true;
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
disp([' J size: ' num2str(m) ' x ' num2str(n)]);
disp(['J nnz: ' num2str(nnz(J_sparse)) ' (' ...
            num2str(100 * nnz(J_sparse) / (m * n), '%04f') '%)']);

RHS_matrix = dlmread(rhs_matrix_file);
QtB_double = dlmread(double_QtB_file);
R_double = ReadSparseMatrix(double_R_file);
p_double = dlmread(double_permutation_file);

% Qtb_float
% QtB_float = dlmread(float_QtB_file);

%% Solve with SPQR.
[QtB_ss, R_ss, p_ss] = spqr(J_sparse, RHS_matrix, 0);
P_ss = speye(size(R_ss, 2));
P_ss = P_ss(:, p_ss);

%% Calculate residuals.
X_ss = R_ss \ QtB_ss; X_ss(p_ss', :) = X_ss;
X_double = R_double \ QtB_double; X_double(p_double, :) = X_double;

%% Compare double --  Matlab's SPQR vs. C++ results.
res_ss = norm(RHS_matrix - J_sparse * X_ss, 'fro');


double_norm_sys = norm(J_sparse' * J_sparse - R_double' * R_double, 'fro');
spqr_sys_norm = norm(J_sparse' * J_sparse - P_ss * R_ss' * R_ss * P_ss', 'fro');
ss_v_double_qtb_norm = norm(QtB_double - QtB_ss, 'fro');

%% Compare float with doubles.

 

%%% Tests
[QtB_ss, R_ss, P_ss] = spqr(J_sparse, RHS_matrix);
X_ss2 = P_ss * (R_ss \ QtB_ss);
res_ss = norm(RHS_matrix - J_sparse * X_ss2, 'fro');



