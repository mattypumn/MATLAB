function points = ReadPointStates( filename )
%READPOINTSTATES Read the saved point states
    fid = fopen(filename);
    
    points.global_ids = ReadIndexVector(fid);
    points.states = ReadMatrix(fid);
    
    fclose(fid);
end

