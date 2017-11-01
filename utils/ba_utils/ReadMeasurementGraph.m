function meas_graph = ReadMeasurementGraph(filename)
%READMEASUREMENTGRAPH Read BA's measurement graph file
    fid = fopen(filename);
    
    meas_graph.meas_size = ReadSingleIndex(fid);
    meas_graph.noise_size = ReadSingleIndex(fid);
    meas_graph.curr_num_meas = ReadSingleIndex(fid);
    
    meas_graph.measurements = ReadMatrix(fid);
    meas_graph.noises = ReadMatrix(fid);
    
    meas_graph.pose2feat = ReadVectorOfArraysOfIndices(fid);
    meas_graph.feat2pose = ReadVectorOfArraysOfIndices(fid);
    meas_graph.feat2pose_meas = ReadVectorOfArraysOfIndices(fid);
    meas_graph.original_ids = ReadIndexVector(fid) + 1;
    
    fclose(fid);
    
    % create a sparse mmtx matrx
    meas_graph.num_features = length(meas_graph.feat2pose);
    meas_graph.num_poses = length(meas_graph.pose2feat);
    
    max_num_meas = size(meas_graph.measurements, 2);
    
    is = zeros(max_num_meas, 1);
    js = zeros(max_num_meas, 1);
    idxs = zeros(max_num_meas, 1);
    curr_num_meas = 1;
    
    % TODO: Change the indices 
    
    for feat_id = 1:meas_graph.num_features
        curr_pose_ids = meas_graph.feat2pose{feat_id};
        
        num_meas = length(curr_pose_ids);
        
        if num_meas == 0
            continue;
        end
        
        is(curr_num_meas:curr_num_meas+num_meas-1) = curr_pose_ids + 1;
        js(curr_num_meas:curr_num_meas+num_meas-1) = feat_id;
        idxs(curr_num_meas:curr_num_meas+num_meas-1) = ...
            meas_graph.feat2pose_meas{feat_id} + 1;
        curr_num_meas = curr_num_meas + num_meas;
    end
    
    meas_graph.mmtx = sparse(is(1:curr_num_meas-1), js(1:curr_num_meas-1), ...
        idxs(1:curr_num_meas-1));
    
end

