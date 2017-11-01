function var_reg = ReadVarReg(filename)
%READVARREG Read the variable registry of the BA
    fid = fopen(filename);
    assert(fid >= 0);
    
    var_reg.jacobian_start_index = ReadIndexVector(fid);
    var_reg.var_enabled = ReadIndexVector(fid);
    
    fclose(fid);
end

