function index = ReadSingleIndex(file_id, type)
%READSINGLEINDEX reads a single index (64-bit unsigened) from the current
%position of the file
    if nargin == 1 || isempty(type)
        type = 'uint64';
    end
    index = fread(file_id, 1, type);
end

