function vector = ReadIndexVector(file_id_or_name, type)
%READINDEXVECTOR Reads an array of values from the current positons of
%file. The first element is interpreted as being the number of elements in
%the array
    if nargin == 1 || isempty(type)
        type = 'uint64';
    end
    if ischar(file_id_or_name)
        file_id = fopen(file_id_or_name);
    else
        file_id = file_id_or_name;
    end

    num_elements = fread(file_id, 1, 'uint64');
    vector = fread(file_id, num_elements, type);
    
    if ischar(file_id_or_name)
        fclose(file_id);
    end
end

