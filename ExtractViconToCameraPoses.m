function [img_nums, vg_p_c, vg_q_c] = ...
        ExtractViconToCameraPoses(vicon_target_file, ...
                                  time_alignment_file, ...
                                  tango_to_vicon_calib_filepath, ...
                                  image_timestamps_file, ...
                                  I_p_C, I_q_C)
    %% Import Data.
    % targ: The reference frame of the vicon target.
    % vg: vicon global
    % tg: tango global
    % _data --> timestamp && (position || orientation)
    % T --> position and orientation.
    % p --> position.
    ext = dlmread(tango_to_vicon_calib_filepath);
    targ_p_imu = ext(1, 1:3)';
    targ_q_imu = ext(2, :)';

    timeshift = dlmread(time_alignment_file);
    vicon_frame_rate = timeshift(1);
    timeshift = timeshift(2);

    vg_T_target_data = dlmread(vicon_target_file);
    v_valid = sum(vg_T_target_data(:,2:end),2) ~= 0;
    vg_T_target_data = vg_T_target_data(v_valid, :);

    % tg_T_imu_data = dlmread(tango_cam_filepath);

    tango_img_timestamps = ReadImageTimestampFile(image_timestamps_file);

    %% Convert Vicon times to Tango.
    vg_T_target_data(:,1) = vg_T_target_data(:,1)/vicon_frame_rate ...
                            + timeshift;

    %% Eliminate any Tango data outside Vicon times.
%     % cond = (tg_T_imu_data(:,1) <= max(vg_T_target_data(:,1))) & ...
%     %         (tg_T_imu_data(:,1) >= min(vg_T_target_data(:,1))); 
%     % tg_T_imu_data = tg_T_imu_data(cond,:);

    %% Millimeters to meters.
    vg_p_targCam = vg_T_target_data(:,2:4) / 1000; 
    %% Prep quaternions for 'slerp'.
    targ1_q_vg = vg_T_target_data(:,5:8);
    diffs = sum(diff(targ1_q_vg),2);
    sign_flag = 1;
    for i=1:length(diffs)
        if abs(diffs(i))>0.1
            sign_flag = -sign_flag;
        end
        targ1_q_vg(i+1,:) = targ1_q_vg(i+1,:)*sign_flag;
    end

    for i=1:length(vg_p_targCam)
        vg_q_targCam(i,:) = quat_inv(targ1_q_vg(i,:)')';
    end

    %% Convert viconGlobal_T_targCam to viconGlobal_T_imuCam
    N = length(vg_p_targCam);
    vg_p_imu_raw = zeros(3, N);
    vg_q_imu_raw = zeros(4, N);
    for i=1:N
        vg_p_imu_raw(:,i) = vg_p_targCam(i,:)' + quat2rot(vg_q_targCam(i,:)') * targ_p_imu;
        vg_q_imu_raw(:,i) = quat_mul(vg_q_targCam(i,:)', targ_q_imu);
    end

    %% Interpolate Vicon Camera poses to align with tango timestamps.
    [vg_p_imu_, vg_q_imu_] = interp_poses(vg_p_imu_raw, vg_q_imu_raw, ...
                                       vg_T_target_data(:,1), tango_img_timestamps(:,2));
                                   
    img_nums = tango_img_timestamps(:,1);
    vg_p_c = [];
    vg_q_c = [];
    for i = 1 : size(vg_p_imu_, 2)
        vg_q_c_i = quat_mul(vg_q_imu_(:,i), I_q_C);
        vg_p_c_i = vg_p_imu_(:, i) + quat2rot(vg_q_imu_(:,i)) * I_p_C;
        
        vg_p_c = [vg_p_c; vg_p_c_i'];
        vg_q_c = [vg_q_c; vg_q_c_i'];
    end

end