%% Parameters.
mat_name = 'J_t_vicon_test_338_4339';
data_generation_dir = '~/RunSuiteSparse/data/';

%% Setup
original_matrix_file = ['/media/mattp/DATA2/SparseMatrices/' mat_name '.dat'];  % Original matrix.
output_file = [data_generation_dir mat_name '_thresholded.dat'];

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
        
        
assert(false && 'There is still no implementation of WriteSparseMatrix in MATLAB');