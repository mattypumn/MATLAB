clear all; clc; close all;

mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

%% Parameters.
data_path = '/usr/local/google/home/mpoulter/for_matt/polaris_floor_vicon/mrinal/1/';

%% Setup.
timestamps_file = [data_path 'dump/feature_tracking_primary_timestamps.txt'];
global_transform_file = [data_path '/Tango_to_Vicon_Calibration.txt'];
time_shift_file = [data_path '/time_alignment.txt'];
vicon_pose_file = [data_path 'target_poses.txt'];
tango_pose_file = [data_path 'out/tango_poses.txt'];

%% Import Data.
ext = dlmread(global_transform_file);
targ_p_imu = ext(1,1:3)';
targ_q_imu = ext(2,:)';
targ_R_imu = quat2rot(targ_q_imu);

timeshift = dlmread([data_path '/time_alignment.txt']);
timeshift = timeshift(2);

vicon_data = dlmread(vicon_pose_file);
v_valid = sum(vicon_data(:,2:end),2) ~= 0;
vicon_data = vicon_data(v_valid, :);
vicon_data(:,1) = vicon_data(:,1)/500 + timeshift;

tango_data = dlmread(tango_pose_file);
cond = (tango_data(:,1) <= max(vicon_data(:,1))) & ...
        (tango_data(:,1) >= min(vicon_data(:,1))); 
tango_data = tango_data(cond,:);


tango_img_time = ReadImageTimestampFile(timestamps_file);

%% Extra prep for vicon poses.
vg_p_targ = vicon_data(:,2:4)/1000;
targ_q_vg = vicon_data(:,5:8);
diffs = sum(diff(targ_q_vg),2);
sign_flag = 1;
for i=1:length(diffs)
    if abs(diffs(i))>0.1
        sign_flag = -sign_flag;
    end
    targ_q_vg(i+1,:) = targ_q_vg(i+1,:)*sign_flag;
end

for i=1:length(vg_p_targ)
    vg_q_targ(i,:) = quat_inv(targ_q_vg(i,:)')';
end

%% Conver to viconGlobal_T_imu
N = length(vg_p_targ);
vg_p_i = zeros(3, N);
vg_q_i = zeros(4, N);
for i=1:N
    vg_p_i(:,i) = vg_p_targ(i,:)' + quat2rot(vg_q_targ(i,:)') * targ_p_imu;
    vg_q_i(:,i) = quat_mul(vg_q_targ(i,:)', targ_q_imu);
end

%% Interpolate Vicon poses to align with tango timestamps.
[vg_p_i_, vg_q_i_] = interp_poses(vg_p_i, vg_q_i, ...
                                        vicon_data(:,1), tango_data(:,1));

%% Extract Vicon Global to Tango global from first time instance.
tg_q_vg = quat_mul(tango_data(1,5:end)', quat_inv(vg_q_i_(:,1)));
tg_R_vg = quat2rot( tg_q_vg);
tg_p_vg =  tango_data(1,2:4)' - tg_R_vg * vg_p_i_(:,1);

%% Transform all vicon measurement to tg_p_imu and tg_q_imu.
N = length(vg_q_i_);
tg_p_i = zeros(3, N);
tg_q_i = zeros(4, N);
for i=1:N
    tg_p_i(:,i) = tg_R_vg * vg_p_i_(:,i) + tg_p_vg;
    tg_q_i(:,i) = quat_mul(tg_q_vg, vg_q_i_(:,i));
end

%% Get corresponding image numbers.
image_nums = [];
for i = 1 : size(tg_p_i, 2)
    num = tango_img_time(abs(tango_img_time(:,2) - tango_data(i,1)) < 1e-6, 1);
    image_nums = [image_nums; num];
end

%% Save interpolated data.
dlmwrite([data_path 'lerp_target_poses.txt'], ... 
        [image_nums, tango_data(:,1), tg_p_i', tg_q_i'], ...
        'delimiter', ' ', 'precision', 15);
