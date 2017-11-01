function vplines = ReadManhattanLineStates(filename)
%READPOINTSTATES Read the saved point states
    fid = fopen(filename);
    assert(fid >= 0);
    
    vplines.global_ids = ReadIndexVector(fid);
    vplines.building_id = ReadSingleIndex(fid);
    vplines.G_C_B = ReadMatrix(fid);
    vplines.classes = ReadIndexVector(fid);
    vplines.states = ReadMatrix(fid);
    
    fclose(fid);
end

 
