function mtx = ReadMatrix(file_id_or_name)
%READMATRIX Reads a matrix stored in a binary file in row-major order.

    filename_provided = ischar(file_id_or_name);

    if filename_provided
        file_id = fopen(file_id_or_name);
        assert(file_id >= 0);
    else
        file_id = file_id_or_name;
    end

    rows = fread(file_id, 1, 'uint64');
    cols = fread(file_id, 1, 'uint64');
    
    num_elements = rows * cols;
    
    values = fread(file_id, num_elements, 'double');
    mtx = reshape(values, cols, rows)';
    
    if filename_provided
        fclose(file_id);
    end
    
end

