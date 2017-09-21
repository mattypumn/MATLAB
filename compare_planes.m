%% Cleanup.
clear; clc; close all;

%% Use functions from MARS.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

%% Params.
% possible programs are {'vio', 'com'}  Is overridden by CALIB_COMPARE_PROGRAM if exists.
desktop_program = 'com';
% set up data directory.  Is overridden by CALIB_COMPARE_DATASET if exists.
base_data_dir = '/usr/local/google/home/mpoulter/for_matt/letango_collection';
vicon_object_file = 'target_poses.txt';

kViconHeightOffset = 0.00;  %  Height of the wand
% NOTE: Per phone call with Vicon, the height of the wand is accounted for
% during calibration.  Thus kViconHeightOffset is set to 0.00.
   
% If this is being ran through command line, these variables should be
% set and we should overwrite the hardcoded values in this script.
% exist CALIB_COMPARE_DATASET var;
% is_running_command_line = ans;
% exist CALIB_COMPARE_PROGRAM var;
% is_running_command_line = is_running_command_line && ans;
% if is_running_command_line
%     base_data_dir = CALIB_COMPARE_DATASET;
%     desktop_program = CALIB_COMPARE_PROGRAM;
% end

datasets = GetSubdirectories(base_data_dir);

rmse_structs = [];
%%  For each dataset in base_data_dir
for data_i = 1 : length(datasets) 
    %% Get subdirectories which are separate runs with different seeds.
    data_path = datasets{data_i};
    disp(data_path);
    iter_folders = GetSubdirectories(fullfile(data_path, 'plane_test_output'));
    dataset_RMSE_1 = [];
    dataset_RMSE_2 = [];

    vicon_pose_file = fullfile(data_path, vicon_object_file);
    time_alignment_file = fullfile(data_path, 'time_alignment.txt');
    tango_to_vicon_calibration = fullfile(data_path, ...
                                'Tango_to_Vicon_Calibration.txt');
    calib_file_1 = fullfile(data_path, 'calibration.xml');
    calib_file_2 = fullfile(data_path, 'average_calibration.xml');   

    for run_i = 1 : length(iter_folders) 
        %% Build filenames.
        plane_folder_1 = fullfile(iter_folders{run_i}, ...
                                [desktop_program '_plane_1']);
        plane_folder_2 = fullfile(iter_folders{run_i}, ...
                                [desktop_program '_plane_2']);
        tango_pose_file_1 = fullfile(iter_folders{run_i}, ...
                          [desktop_program '_tango_1'], 'tango_poses.txt');
        tango_pose_file_2 = fullfile(iter_folders{run_i}, ...
                          [desktop_program '_tango_2'], 'tango_poses.txt');    

        
        %% Build Data structs.
        plane_struct_1 = struct();
        plane_struct_1.plane_folder = plane_folder_1;
        plane_struct_1.vicon_pose_file = vicon_pose_file;
        plane_struct_1.tango_pose_file = tango_pose_file_1;
        plane_struct_1.time_alignment_file = time_alignment_file;
        plane_struct_1.calibration_file = calib_file_1;
        plane_struct_1.tango_to_vicon_calibration = tango_to_vicon_calibration;
        plane_struct_1.data_path = data_path;
        
        plane_struct_2 = struct();
        plane_struct_2.plane_folder = plane_folder_2;
        plane_struct_2.vicon_pose_file = vicon_pose_file;
        plane_struct_2.tango_pose_file = tango_pose_file_2;
        plane_struct_2.time_alignment_file = time_alignment_file;
        plane_struct_2.calibration_file = calib_file_2;
        plane_struct_2.tango_to_vicon_calibration = tango_to_vicon_calibration;        
        plane_struct_2.data_path = data_path;
        
        %% Analyze and save.
        dataset_RMSE_1 = [dataset_RMSE_1, ...
                        AnalyzePlaneDataForRMSE(plane_struct_1)];
        dataset_RMSE_2 = [dataset_RMSE_2, ...
                        AnalyzePlaneDataForRMSE(plane_struct_2)];
        disp(['iter: ' num2str(run_i) ' ' num2str(dataset_RMSE_1(end)) ' ' num2str(dataset_RMSE_2(end))]);
    end
    
    dataset_RMSE_1 = dataset_RMSE_1(dataset_RMSE_1 < Inf);
    dataset_RMSE_2 = dataset_RMSE_2(dataset_RMSE_2 < Inf);
    
    if isempty(dataset_RMSE_1)
        dataset_RMSE_1 = Inf;
    end
    if isempty(dataset_RMSE_2)
        dataset_RMSE_2 = Inf;
    end
    
    
    %% Save RMSE information for dataset.
    results = struct();
    results.data_dir = data_path;

    results.mean_1 = mean(dataset_RMSE_1);
    results.stddev_1 = std(dataset_RMSE_1);

    results.mean_2 = mean(dataset_RMSE_2);
    results.stddev_2 = std(dataset_RMSE_2);
    
    rmse_structs = [rmse_structs; results];
    disp(['finished ' num2str(data_i) ': ' num2str(results.mean_1)  ' ' num2str(results.mean_2)]);
end

pause;


%% Plot Floor heights in Tango Global.
tango_fig = figure();
hold on; grid on;
ylim([0 2]);
plot(t_times_1, tg_heights_1, '+b');
plot(t_times_2, tg_heights_2, 'og');
plot(tango_combined_timestamps, repmat(-tg_p_vg_1(3) + kViconHeightOffset, size(tango_combined_timestamps,1),1), '-r');
title('Dominant Floor Plane Heights in Tango Global')
legend([desktop_program ': 1'], [desktop_program ': 2'], 'Vicon');
xlabel('time (s)'); ylabel('floor height (m)');
hold off;

%% Plot Heights in IMU frame.
imu_fig = figure();
hold on; grid on;
ylim([0 2]);
plot(t_times_1, imu_heights_1, '+b');
plot(t_times_2, imu_heights_2, 'og');
plot(tango_combined_timestamps, vicon_floor2Imu_in_tangoFrame(:,3) + kViconHeightOffset, '-r');
title('Dominant Floor Plane Heights in Tango Global')
legend([desktop_program ': 1'], [desktop_program ': 2'], 'Vicon');
xlabel('time (s)'); ylabel('floor height (m)');



%% Helper Functions
function [tg_q_i, tg_p_i, tango_time] = ExtractTangoData(tango_poses_file)
    tango_traj = dlmread(tango_poses_file);
    tango_time = tango_traj(:,1);
    if ~isempty(tango_time)
       tg_p_i = tango_traj(:,2:4);
       tg_q_i = tango_traj(:,5:8);
    else 
        tg_p_i = [];
        tg_q_i = [];
    end
end


function [plane_data] = ExtractPlaneData(directory)
    heights = [];
    timestamps = [];
    kMinFloorMeasurements = 20;
    files = GetAllFiles(directory);
    %% Exctract data.
    plane_data = [];
    for i = 1 : size(files, 1)
        file_str = files{i};
        s = dir(file_str);
        if s.bytes == 0
            continue;
        else
            % open the file and read it
            tmp_data = dlmread(file_str);
            %  Make sure the data is valid.
            tmp_data = tmp_data(tmp_data(:,5) > 0, :);
            plane_data = [plane_data; tmp_data];
        end
    end
    
    if isempty(plane_data) 
        return;
    end
    
    %% Sort Data.
    [~, order_by_updated_time] = sort(plane_data(:,2));
    plane_data = plane_data(order_by_updated_time, :);
end




function [heights, timestamps] = ExtractFloorHeights(directory)
    heights = [];
    timestamps = [];
    kMinFloorMeasurements = 20;
    files = GetAllFiles(directory);
    %% Exctract data.
    data = [];
    for i = 1 : size(files, 1)
        file_str = files{i};
        s = dir(file_str);
        if s.bytes == 0
            continue;
        else
            % open the file and read it
            tmp_data = dlmread(file_str);
            %  Make sure the data is valid.
            tmp_data = tmp_data(tmp_data(:,5) > 0, :);
            data = [data; tmp_data];
        end
    end
    
    if isempty(data) 
        return;
    end
    
    %% Sort Data.
    [~, order_idx] = sort(data(:,2));
    data = data(order_idx, :);
    
    %% Find floor.
    while ~isempty(data)
        max_height = max(data(:,4));
        id_of_max = data(data(:,4) == max_height, 1);  % may return multiple.
        id_of_max = id_of_max(1);  % Lets deal with the first.
        idxs_of_max_id = data(:,1) == id_of_max;
        max_id_count = sum(idxs_of_max_id);
        if max_id_count > kMinFloorMeasurements
            %% Found our floor.
            heights = data(idxs_of_max_id, 4);
            timestamps = data(idxs_of_max_id, 2:3);
            return;
        else 
            %% This is noise. Get rid of this id.
            data = data(~idxs_of_max_id,:);
        end
    end
end


function [vg_q_targ, vg_p_targ, vicon_time] = ExtractViconData(vicon_file)
    % Returned format:   [vector1'; vector2'; ...; vectorN]
    vg_q_targ = []; vg_p_targ = []; vicon_time = [];
    data = dlmread(vicon_file);
    v_valid = sum(data(:,2:end),2) ~= 0;
    data = data(v_valid, :);
    vicon_time = data(:,1);
    
    vg_p_targ = data(:,2:4)/1000;
    targ_q_vg = data(:,5:8);
    diffs = sum(diff(targ_q_vg),2);
    sign_flag = 1;
    for i=1:length(diffs)
        if abs(diffs(i))>0.1
            sign_flag = -sign_flag;
        end
        targ_q_vg(i+1,:) = targ_q_vg(i+1,:)*sign_flag;
    end

    for i=1:size(vg_p_targ, 1)
        vg_q_targ(i,:) = quat_inv(targ_q_vg(i,:)')';
    end
end

function times = CombineTimestamps(tango_time_1, tango_time_2)
    %  Union the set of timestamps and return in sorted order.
    times = [tango_time_1', tango_time_2']';
    times = sort(times);
    bools = logical(ones(size(times,1), 1));
    for i = 2 : length(times)
        if times(i) == times(i-1)
            bools(i) = false;
        end
    end
    times = times(bools);
end

function fileList = GetAllFiles(dirName)
  dirData = dir(dirName);      %# Get the data for the current directory
  dirIndex = [dirData.isdir];  %# Find the index for directories
  fileList = {dirData(~dirIndex).name}';  %'# Get a list of the files
  if ~isempty(fileList)
    fileList = cellfun(@(x) fullfile(dirName,x),...  %# Prepend path to files
                       fileList,'UniformOutput',false);
  end
  subDirs = {dirData(dirIndex).name};  %# Get a list of the subdirectories
  validIndex = ~ismember(subDirs,{'.','..'});  %# Find index of subdirectories
                                               %#   that are not '.' or '..'
  for iDir = find(validIndex)                  %# Loop over valid subdirectories
    nextDir = fullfile(dirName,subDirs{iDir});    %# Get the subdirectory path
    fileList = [fileList; GetAllFiles(nextDir)];  %# Recursively call getAllFiles
  end
end


function directories = GetSubdirectories(base_data_dir)
    tmp_datasets = dir (base_data_dir);
    directories = {};
    for i = 1 : length(tmp_datasets)
        folder = tmp_datasets(i);
        if strcmp(folder.name, '.') == 1
            continue;
        elseif strcmp(folder.name, '..') == 1
            continue;
        elseif ~(folder.isdir)
            continue;
        else 
            directories{end + 1} = fullfile(base_data_dir, folder.name); 
        end
    end
end


function [i_p_c, i_q_c] = ExctractIntrinsics(xml_file)
    kImuId = 0;
    kCameraId = 100;
    disp(xml_file);
%     [i_p_c, i_q_c] = ParseCalibrationXml(file, kImuId, kCameraId);
    i_p_c = 0; i_q_c = 0;
end



function [RMSE] = AnalyzePlaneDataForRMSE(data_struct)
    %% File to write meta data.
    [path, name, ~] = fileparts(data_struct.plane_folder);
    write_file = fullfile(path, [name '_vicon_and_tango_heights.txt']);
    if exist(write_file, 'file') == 2
        s = dir(write_file);
        if s.bytes == 0
            delete(write_file);
        else
            data = dlmread(write_file); 
            err = data(:,2)- data(:,3);
            RMSE = sqrt(mean(err.^2));
            return;
        end
    end

    %% Extract raw data from files.
    timing_alignment = dlmread(data_struct.time_alignment_file);

    [tg_q_i, tg_p_i, tango_times] = ...
                        ExtractTangoData(data_struct.tango_pose_file);

    [heights, plane_times] = ExtractPlaneData(data_struct.plane_folder);
    if isempty(plane_times)
%         disp('No measured planes!');
        RMSE = Inf;
        return;
    end

    [vg_q_targ, vg_p_targ, vicon_time] = ...
                    ExtractViconData(data_struct.vicon_pose_file);

    % tango_to_vicon alignment.
    ext = dlmread(data_struct.tango_to_vicon_calibration);
    targ_p_imu = ext(1,1:3)';
    targ_q_imu = ext(2,:)';

    % Calibration.
%     [i_p_c, i_q_c] = ExctractIntrinsics(data_struct.calibration_file);

    
    %% Sync data.
    vicon_time = vicon_time/timing_alignment(1) + timing_alignment(2);

    cond = (tango_times <= max(vicon_time)) & ...
            (tango_times >= min(vicon_time)); 
    tango_times = tango_times(cond);

    [vg_p_targ_, vg_q_targ_] = interp_poses(vg_p_targ', vg_q_targ', ...
                                     vicon_time, tango_times);
    if isempty(vg_p_targ_) 
        disp('Empty set returned from interp_poses.   Exiting...');
        RMSE = Inf;
        return;
    end    
    
    %%%%%%%%%%%%%%%%%%%%%   {FRAME} Conversions  %%%%%%%%%%%%%%%%%%%%                             
    %% Convert viconGlobal_T_target >>> viconGlobal_T_imu
    N = size(vg_p_targ_, 2);
    vg_p_i_ = zeros(N, 3);
    vg_q_i_ = zeros(N, 4);
    for time_i=1:N
        vg_p_i_(time_i,:) = (vg_p_targ_(:,time_i) + ...
                            quat2rot(vg_q_targ_(:,time_i)) * targ_p_imu)';
        vg_q_i_(time_i,:) = quat_mul(vg_q_targ_(:,time_i), targ_q_imu)';
    end

    %% Extract Vicon Global to Tango global from first time instance.
    tg_q_vg = quat_mul(tg_q_i(1,:)', quat_inv(vg_q_i_(1,:)'));
    tg_R_vg = quat2rot(tg_q_vg);
    tg_p_vg =  tg_p_i(1,:)' - tg_R_vg * vg_p_i_(1,:)';

    %% Consider Vicon y-axis to be an absolute height.
%     vicon_floor2Imu_in_tangoFrame = (tg_R_vg * vg_p_i_')';
    vicon_imu_heights = vg_p_i_(:,2);

    %%  Extract heights.
    imu_TangoHeights= [];
    imu_ViconHeights = [];
    times = [];
    for time_i = 1 : size(tango_times,1)
       kTimeEpsilon = 1e-6;
       height_idxs = find(abs(plane_times(:,1) - tango_times(time_i)) < kTimeEpsilon);

       if isempty(height_idxs)
           continue;
       end
       
       tg_p_i_timei = (tg_p_i(time_i,:))';
       
       tg_heights = heights(height_idxs);
       
       if ~isempty(tg_heights)
          % Sanity check.
          if ~all(tg_heights == tg_heights(1))
              disp('Error with one of the tango_heights');
              tg_heights
             pause; 
             continue;
          end
          
          imu_floorheight_FromTango = tg_heights(1) + tg_p_i_timei(3);
          imu_floorheight_FromVicon = vicon_imu_heights(time_i);
            
          imu_TangoHeights = [imu_TangoHeights; imu_floorheight_FromTango];
          imu_ViconHeights = [imu_ViconHeights; imu_floorheight_FromVicon];
          times = [times; tango_times(time_i)];
       end
    end
    %% Save Heights.
    dlmwrite(write_file, [times, imu_TangoHeights, imu_ViconHeights]);
    
    %% Calculate RMSE.
    if ~isempty(imu_TangoHeights) && ~isempty(imu_ViconHeights)
        err = abs(imu_TangoHeights - imu_ViconHeights);
        RMSE = sqrt(mean(err.^2));
    else  
        RMSE = Inf;
    end
end

