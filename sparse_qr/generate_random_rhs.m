%% Parameters.
mat_name = '5R_600cols';
data_generation_dir = '~/RunSuiteSparse/data/';
kTransposeOriginalMatrix = false;
kNumRhsColumns = 3;

%% Setup
original_matrix_file = ['/media/mattp/DATA2/SparseMatrices/' mat_name '.dat'];  % Original matrix.
output_file = [data_generation_dir mat_name '_random_rhs.txt'];

%% Add path to utils.
addpath('~/MATLAB/utils/ba_utils/');

%% Load Data.
disp(['Loading matrix: ' mat_name]);
J_sparse = ReadSparseMatrix(original_matrix_file);
if kTransposeOriginalMatrix 
    J_sparse = J_sparse';
end
[m, n] = size(J_sparse);
disp(['J size: ' num2str(m) ' x ' num2str(n)]);
disp(['J nnz: ' num2str(nnz(J_sparse)) ' (' ...
            num2str(100 * nnz(J_sparse) / (m * n), '%04f') '%)']);
        
%% Generate RHS.
B = rand(m, kNumRhsColumns);

%% Test.
disp('Solving system to test for errors...');
X = J_sparse \ B;  %%  Should raise alarm if the system cannot be solved.

disp('Solving sub-system to test for errors...');
X = J_sparse(:,1:600) \ B;  %%  Should raise alarm if the system cannot be solved.

%% Save RHS.
dlmwrite(output_file, B);