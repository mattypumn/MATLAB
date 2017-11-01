function mat = ReadSparseMatrix(filename)
%READSPARSEMATRIX Reads a sparse matrix from a binary file
    fid = fopen(filename);
    
    num_rows = ReadSingleIndex(fid);
    num_cols = ReadSingleIndex(fid);
    nnz = ReadSingleIndex(fid);
    
    rows = ReadIndexVector(fid);
    cols = ReadIndexVector(fid);
    vals = ReadIndexVector(fid, 'double');
    
    assert(length(rows) == nnz);
    assert(length(cols) == nnz);
    assert(length(vals) == nnz);
    
    mat = sparse(rows + 1, cols + 1, vals, num_rows, num_cols, nnz);
    
    fclose(fid);
end

