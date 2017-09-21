%% Cleanup.
clear; clc; close all;

%% Params.
% possible programs are {'vio', 'com'}  Is overridden by CALIB_COMPARE_PROGRAM if exists.
desktop_program = 'com';
% set up data directory.  Is overridden by CALIB_COMPARE_DATASET if exists.
data_base_dir = '/usr/local/google/home/mpoulter/for_matt/tmp/test';
data_sub_dir = '';
vicon_object_file = 'target_poses.txt';
kViconGlobalOffset = .02;

% TODO  Have calibration files parsed.
% calib_file_1 = '';
% calib_file_2 = '';
% Camera Extrinsics.
i1_p_c1 = [0.004718641446412659;  0.02439645089534794;  -0.02686148616790133] ;
i1_q_c1 = [-0.9998604612232053;  0.003304862190576937;  -0.007085454619255722;  0.01476253031356873] ;
i2_p_c2 = [0.004718641446412659;  0.02439645089534794;  -0.02686148616790133] ;
i2_q_c2 = [-0.9998604612232053;  0.003304862190576937;  -0.007085454619255722;  0.01476253031356873] ;

%% Setup.
data_path = fullfile(data_base_dir, data_sub_dir);
exist CALIB_COMPARE_DATASET var;
is_running_command_line = ans;
exist CALIB_COMPARE_PROGRAM var;
is_running_command_line = is_running_command_line && ans;
if is_running_command_line
    data_sub_dir = CALIB_COMPARE_DATASET;
    desktop_program = CALIB_COMPARE_PROGRAM;
end
plane_folder_1 = fullfile(data_path, 'plane_test_output', 'iter_1',  [desktop_program '_plane_1']);
plane_folder_2 = fullfile(data_path, 'plane_test_output', 'iter_1', [desktop_program '_plane_2']);
time_alignment_file = fullfile(data_path, 'time_alignment.txt');
tango_to_vicon_cal = fullfile(data_path, 'Tango_to_Vicon_Calibration.txt');
vicon_pose_file = fullfile(data_path, vicon_object_file);
tango_pose_file_1 = fullfile(data_path, 'plane_test_output', 'iter_1', [desktop_program '_tango_1'], 'tango_poses.txt');
tango_pose_file_2 = fullfile(data_path, 'plane_test_output', 'iter_1', [desktop_program '_tango_2'], 'tango_poses.txt');
% TODO implement below function.
% [i1_p_c1, i1_q_c1] = ParseCalibrationFile(calib_file_1);
% [i2_p_c2, i2_q_c2] = ParseCalibrationFile(calib_file_2);

%% Use functions from MARS.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

%% Extract data.
timing_data = dlmread(time_alignment_file);

[tg_q_i_1, tg_p_i_1, tango_time_1] = ExtractTangoData(tango_pose_file_1);
[tg_q_i_2, tg_p_i_2, tango_time_2] = ExtractTangoData(tango_pose_file_2);

[heights_1, plane_times_1] = ExtractFloorHeights(plane_folder_1);
[heights_2, plane_times_2] = ExtractFloorHeights(plane_folder_2);

[vg_q_targ, vg_p_targ, vicon_time] = ExtractViconData(vicon_pose_file);

% tango_to_vicon alignment.
ext = dlmread(tango_to_vicon_cal);
targ_p_imu = ext(1,1:3)';
targ_q_imu = ext(2,:)';

%% Sync data.
vicon_time = vicon_time/timing_data(1) + timing_data(2);1

tango_combined_timestamps = CombineTimestamps(tango_time_1, tango_time_2);
cond = (tango_combined_timestamps <= max(vicon_time)) & ...
        (tango_combined_timestamps >= min(vicon_time)); 
tango_combined_timestamps = tango_combined_timestamps(cond);

[vg_p_targ_, vg_q_targ_] = interp_poses(vg_p_targ', vg_q_targ', ...
                                 vicon_time, tango_combined_timestamps);
if isempty(vg_p_targ) 
    disp('Empty set returned from interp_poses.   Exiting...');
    return;
end
                             
%%%%%%%%%%%%%%%%%%%%%   {FRAME} Conversions  %%%%%%%%%%%%%%%%%%%%                             
%% Convert viconGlobal_T_target >>> viconGlobal_T_imu
N = size(vg_p_targ_, 2);
vg_p_i_ = zeros(N, 3);
vg_q_i_ = zeros(N, 4);
for i=1:N
    vg_p_i_(i,:) = (vg_p_targ_(:,i) + quat2rot(vg_q_targ_(:,i)) * targ_p_imu)';
    vg_q_i_(i,:) = quat_mul(vg_q_targ_(:,i), targ_q_imu)';
end


%% Extract Vicon Global to Tango global from first time instance.
tg_q_vg_1 = quat_mul(tg_q_i_1(1,:)', quat_inv(vg_q_i_(1,:)'));
tg_R_vg_1 = quat2rot(tg_q_vg_1);
tg_p_vg_1 =  tg_p_i_1(1,:)' - tg_R_vg_1 * vg_p_i_(1,:)';

tg_q_vg_2 = quat_mul(tg_q_i_2(1,:)', quat_inv(vg_q_i_(1,:)'));
tg_R_vg_2 = quat2rot(tg_q_vg_2);
tg_p_vg_2 =  tg_p_i_2(1,:)' - tg_R_vg_2 * vg_p_i_(1,:)';

%% Convert vicon height to Tango heights.
% vicon_Floor2Imu = [zeros(size(vg_p_i_(:,2))) vg_p_i_(:, 2), zeros(size(vg_p_i_(:,2)))]; 
vicon_floor2Imu_in_tangoFrame = (tg_R_vg_1 * vg_p_i_')'; vicon_floor2Imu_in_tangoFrame(:,3);

%%
tg_heights_1 = [];
c_heights_1 = [];
t_times_1 = [];
tg_heights_2 = [];
c_heights_2 = [];
t_times_2 = [];
for i = 1 : size(tango_combined_timestamps,1)
   kTimeEpsilon = 1e-6;
   height_idxs_1 = find(abs(plane_times_1(:,1) - tango_combined_timestamps(i)) < kTimeEpsilon);
   height_idxs_2 = find(abs(plane_times_2(:,1) - tango_combined_timestamps(i)) < kTimeEpsilon);
   tango_idx_1 = find(abs(tango_combined_timestamps(i) - tango_time_1 ) < kTimeEpsilon);
   tango_idx_2 = find(abs(tango_combined_timestamps(i) - tango_time_2 ) < kTimeEpsilon);
   
   h_1 = heights_1(height_idxs_1);
   h_2 = heights_2(height_idxs_2);
   
   if ~isempty(h_1)
      % Sanity check.
      assert(all(h_1 == h_1(1))); 
      assert (~isempty(tango_idx_1));
      % Save height in {Tango_global}.
      tg_heights_1 = [tg_heights_1; h_1(1)];  
      t_times_1 = [t_times_1; tango_combined_timestamps(i)];
      
      % Save height in {IMU}
      tg_p_i = tg_p_i_1(tango_idx_1, :);
      c_heights_1 = [c_heights_1; -tg_p_vg_1(3) + tg_p_i(3)];
   end
   
   if ~isempty(h_2)
      % Sanity check.
      assert(all(h_2 == h_2(1)));
      assert (~isempty(tango_idx_2));
      % Save height in {Tango_global}.
      tg_heights_2 = [tg_heights_2; h_2(1)];
      t_times_2 = [t_times_2; tango_combined_timestamps(i)];
      
      % Save heigth in {IMU}
      tg_p_i = tg_p_i_1(tango_idx_1, :);
      c_heights_2 = [c_heights_2; -tg_p_vg_2(3) + tg_p_i(3)];
   end   
   disp(num2str(i));
end

%% Plot Floor heights in Tango Global.
tango_fig = figure();
hold on; grid on;
ylim([0 2]);
plot(t_times_1, tg_heights_1, '+b');
plot(t_times_2, tg_heights_2, 'og');
plot(tango_combined_timestamps, repmat(-tg_p_vg_1(3) + kViconGlobalOffset, size(tango_combined_timestamps,1),1), '-r');
title('Dominant Floor Plane Heights in Tango Global')
legend([desktop_program ': 1'], [desktop_program ': 2'], 'Vicon');
xlabel('time (s)'); ylabel('floor height (m)');
hold off;

%% Plot Heights in IMU frame.
imu_fig = figure();
hold on; grid on;
ylim([0 2]);
plot(t_times_1, c_heights_1, '+b');
plot(t_times_2, c_heights_2, 'og');
plot(tango_combined_timestamps, vicon_floor2Imu_in_tangoFrame(:,3) + kViconGlobalOffset, '-r');
title('Dominant Floor Plane Heights in {IMU}')
legend([desktop_program ': 1'], [desktop_program ': 2'], 'Vicon');
xlabel('time (s)'); ylabel('floor height (m)');

%% Calculate RMSE.
err_1 = abs(v_h-t_h1);
rmse_fh1 = sqrt(mean(err_1(t_h1 > 1).^2));

err_2 = abs(v_h-t_h2);
rmse_fh2 = sqrt(mean(err_2(t_h2 > 1).^2));

disp(['Floor height RMSE_1 (m): ' num2str(rmse_fh1)])
disp(['Floor height RMSE_2 (m): ' num2str(rmse_fh2)])


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

function [heights, timestamps] = ExtractFloorHeights(directory)
    heights = [];
    timestamps = [];
    kMinFloorMeasurements = 20;
    files = getAllFiles(directory);
    data = [];
    for i = 1 : size(files, 1)
        file_str = files{i};
        s = dir(file_str);
        if s.bytes == 0
            continue;
        else
            % open the file and read it
            tmp_data = dlmread(file_str);
            tmp_data = tmp_data(tmp_data(:,5) > 0, :);
            data = [data; tmp_data];
        end
    end
    
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




function [i_p_c, i_q_c] = ExtractIntrinsics(xml_file)
    %% Not yet implemented.
    disp('ExtractIntrinsics is not yet implemented\n');
    disp([xml_file, '\n']);
    i_p_c = [];
    i_q_c = [];
end


function fileList = getAllFiles(dirName)
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
    fileList = [fileList; getAllFiles(nextDir)];  %# Recursively call getAllFiles
  end
end