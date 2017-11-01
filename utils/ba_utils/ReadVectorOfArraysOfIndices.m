function array_cell = ReadVectorOfArraysOfIndices(file_id_or_name)
%READVECTOROFARRAYSOFINDICES Reads a
%std::vector<std::vector<std::uint64_t>>

    if ischar(file_id_or_name)
        file_id = fopen(file_id_or_name);
    else
        file_id = file_id_or_name;
    end

    num_vectors = ReadSingleIndex(file_id);
    array_cell = cell(num_vectors, 1);
    
    for cell_id = 1:num_vectors
        array_cell{cell_id} = ReadIndexVector(file_id);
    end
    
    if ischar(file_id_or_name)
        fclose(file_id);
    end
end

