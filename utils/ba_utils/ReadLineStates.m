function lines = ReadLineStates(filename)
%READPOINTSTATES Read the saved point states
    fid = fopen(filename);
    assert(fid >= 0);
    
    lines.global_ids = ReadIndexVector(fid);
    lines.states = ReadMatrix(fid);
    assert(size(lines.states, 1) == 5);
    
    fclose(fid);
end

 
 
