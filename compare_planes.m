%% Cleanup.
clear all; clc; close all;

%% Params.
% possible programs are {'vio', 'com'}
desktop_program = 'vio';
data_base_dir = '/usr/local/google/home/mpoulter/for_matt/polaris_floor_vicon/mrinal/';
data_sub_dir = '4';
vicon_object_file = 'target_poses.txt';

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
plane_folder_1 = fullfile(data_path, [desktop_program '_plane_1']);
plane_folder_2 = fullfile(data_path, [desktop_program '_plane_2']);
time_alignment_file = fullfile(data_path, 'time_alignment.txt');
tango_to_vicon_cal = fullfile(data_path, 'Tango_to_Vicon_Calibration.txt');
vicon_pose_file = fullfile(data_path, vicon_object_file);
tango_pose_file_1 = fullfile(data_path, [desktop_program '_tango_1'], 'tango_poses.txt');
tango_pose_file_2 = fullfile(data_path, [desktop_program '_tango_2'], 'tango_poses.txt');
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
vicon_time = vicon_time/timing_data(1) + timing_data(2);

tango_combined_timestamps = CombineTimestamps(tango_time_1, tango_time_2);
cond = (tango_combined_timestamps <= max(vicon_time)) & ...
        (tango_combined_timestamps >= min(vicon_time)); 
tango_combined_timestamps = tango_combined_timestamps(cond);

[vg_p_targ_, vg_q_targ_] = interp_poses(vg_p_targ, vg_q_targ, ...
                                 vicon_time', tango_combined_timestamps');

%% Extract Vicon Global to Tango global from first time instance.
tg_q_vg = quat_mul(tg_q_i_1(:,1), quat_inv(vg_q_i_(:,1)));
tg_R_vg = quat2rot(tg_q_vg);
tg_p_vg =  tg_p_i_1(:,1) - tg_R_vg * vg_p_i_(:,1);

%% Convert to proper frames.
% Convert to viconGlobal_T_imu
N = length(vg_p_targ_);
vg_p_i_ = zeros(3, N);
vg_q_i_ = zeros(4, N);
for i=1:N
    vg_p_i_(:,i) = vg_p_targ_(:,i) + quat2rot(vg_q_targ_(:,i)) * targ_p_imu;
    vg_q_i_(:,i) = quat_mul(vg_q_targ_(:,i), targ_q_imu);
end

v_heights = vg_p_i_(3, :);
t_heights_1 = [];
t_times_1 = [];
t_heights_2 = [];
t_times_2 = [];
for i = 1 : size(tango_combined_timestamps,2)
   idxs_1 = find(plane_times_1 == tango_combined_timestamps(i));
   idxs_2 = find(plane_times_2 == tango_combined_timestamps(i));
   
   h_1 = heights_1(idxs_1);
   h_2 = heights_2(idxs_2);
   
   if ~isempty(h_1)
      % Sanity check.
      assert(all(h_1 == h_1(1))); 
      % Save.
      t_heights_1 = [t_height_1, h_1(1)];  
      t_times_1 = [t_times_1, tango_combined_timestamps(i)];
   end
   
   if ~isempty(h_2)
      % Sanity check.
      assert(all(h_2 == h_2(1)));
      % Save.
      t_heights_2 = [t_height_2, h_2(1)];
      t_times_2 = [t_times_2, tango_combined_timestamps(i)];
   end   
end

%% Plot synchro-ed data.
figure();
hold on; grid on;
plot(time_h, t_h1, '+b');
plot(time_h, t_h2, 'xg');
plot(time_h, v_heights, 'or');
title('Floor height estimate')
legend([desktop_program ': 1'], [desktop_program ': 2'], 'Vicon');
xlabel('time (s)'), ylabel('floor height (m)')
err_1 = abs(v_h-t_h1);
rmse_fh1 = sqrt(mean(err_1(t_h1 > 1).^2));

err_2 = abs(v_h-t_h2);
rmse_fh2 = sqrt(mean(err_2(t_h2 > 1).^2));

disp(['Floor height RMSE_1 (m): ' num2str(rmse_fh1)])
disp(['Floor height RMSE_2 (m): ' num2str(rmse_fh2)])


%% Helper Functions
function [tg_q_i, tg_p_i, tango_time] = ExtractTangoData(tango_poses)
    disp([tango_poses, 'ExtractTangoData not implemented']);
    tg_q_i = [];
    tg_p_i = [];
    tango_time = [];
end

function [heights, timestamps] = ExtractFloorHeights(directory)
    kMinFloorMeasurements = 10;
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
    data = dlmread(vicon_file);
    v_valid = sum(data(:,2:end),2) ~= 0;
    data = data(v_valid, :);
    vicon_time = data(:,1)';
    
    vg_p_targ = data(:,2:4)'/1000;
    targ_q_vg = data(:,5:8);
    diffs = sum(diff(targ_q_vg),2);
    sign_flag = 1;
    for i=1:length(diffs)
        if abs(diffs(i))>0.1
            sign_flag = -sign_flag;
        end
        targ_q_vg(i+1,:) = targ_q_vg(i+1,:)*sign_flag;
    end

    for i=1:length(vg_p_targ)
        vg_q_targ(:,i) = quat_inv(targ_q_vg(i,:)');
    end
end

function times = CombineTimestamps(tango_time_1, tango_time_2)
    times = [tango_time_1, tango_time_2];
    times = sort(times);
    bools = logical(ones(1,size(times,2)));
    for i = 2 : size(times,2)
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