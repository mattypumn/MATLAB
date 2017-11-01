function poses = ReadPosesFile(filename)
%READPOSESFILE Read the binary file resulting from saving the poses
    fid = fopen(filename);
    
    if fid < 0
        disp(['Unable to open file ',filename, '.']);
        assert(fid >= 0);
    end
    
    poses.global_ids = ReadIndexVector(fid);
    poses.rolling_shutter_var_id = ReadSingleIndex(fid);
    poses.imu_cam_extrinsics_id = ReadSingleIndex(fid);
    poses.imu_intrinsics_id = ReadSingleIndex(fid);
    
    poses.states = ReadMatrix(fid);
    poses.imu_q_cam = ReadMatrix(fid);
    poses.imu_p_cam = ReadMatrix(fid);
    poses.rolling_shutter_time = fread(fid, 1, 'double');
    poses.a_q_g = ReadMatrix(fid);
    poses.S_g = ReadMatrix(fid);
    poses.D_g = ReadMatrix(fid);
    poses.S_a = ReadMatrix(fid);
    poses.D_a = ReadMatrix(fid);
    poses.K_a = ReadMatrix(fid);
    
    fclose(fid);
end

