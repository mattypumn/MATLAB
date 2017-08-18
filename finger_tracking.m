clear; clc; close all;

%% Using MARS Matlab.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

%% Parameters.
% base directory
DEBUG_IMAGES = false;
bounding_pixel_radius = 60;
start_image = 6;
end_image = 10000;
dataset_dir = '~/for_matt/pixel_finger/exp7/';
tango_to_vicon_calib_filepath = ['~/for_matt/pixel_finger/exp1/','Tango_to_Vicon_Calibration.txt'];
vicon_frame_rate = 500;
kMaxTimeDiffBetweenCamPoseAndFingerPosition = 1 / vicon_frame_rate;
I_q_C = [-0.9999381563640312;  0.004447440852834265;  -0.009283474950362221;  0.004209609246604872];
I_p_C =  [-0.008010455819607585;  0.00785687880804543;  -0.008900038558237013];
fc = [480.9527955850893;  480.74901328696];
cc = [319.3068719746725; 242.9280558633993];
kc = [0.01613255596460211; -0.06416463599461311; 0; 0; 0.1025574979617985];

%% Setup.
vicon_cam_filepath = fullfile(dataset_dir, 'axe_pose.txt');
vicon_finger_filepath = fullfile(dataset_dir, 'untracked_position.txt');
tango_cam_filepath = fullfile(dataset_dir, 'out/tango_poses.txt');
time_alignment_filepath = fullfile(dataset_dir, 'time_alignment.txt');
image_timestamps_file = fullfile(dataset_dir, 'dump/feature_tracking_primary_timestamps.txt');
image_dir = fullfile(dataset_dir, 'dump/feature_tracking_primary/');
box_output_file = fullfile(dataset_dir, 'dump/finger_bounding_box.txt');
marker_output_file = fullfile(dataset_dir, 'dump/marker_pixels.txt');
box_format_file = fullfile(dataset_dir, 'box_output_readme.txt');


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
targ_R_imu = quat2rot(targ_q_imu);

timeshift = dlmread(time_alignment_filepath);
timeshift = timeshift(2);

vg_T_target_data = dlmread(vicon_cam_filepath);
v_valid = sum(vg_T_target_data(:,2:end),2) ~= 0;
vg_T_target_data = vg_T_target_data(v_valid, :);

vg_p_finger_data = dlmread(vicon_finger_filepath);
v_valid = sum(vg_p_finger_data(:,2:end),2) ~= 0;
vg_p_finger_data = vg_p_finger_data(v_valid, :);

tg_T_imu_data = dlmread(tango_cam_filepath);

tango_img_timestamps = ReadImageTimestampFile(image_timestamps_file);

%% Convert Vicon times to Tango.
vg_T_target_data(:,1) = vg_T_target_data(:,1)/vicon_frame_rate + timeshift;
vg_p_finger_data(:,1) = vg_p_finger_data(:,1)/vicon_frame_rate + timeshift;

%% Eliminate any Tango data outside Vicon times.
cond = (tg_T_imu_data(:,1) <= max(vg_T_target_data(:,1))) & ...
        (tg_T_imu_data(:,1) >= min(vg_T_target_data(:,1))); 
tg_T_imu_data = tg_T_imu_data(cond,:);

%% Extra prep for vicon poses.
vg_p_targCam = vg_T_target_data(:,2:4)/1000;
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

% There are no quaternions in untracked markers.  Millimeters to Meters.
vg_p_finger_data(:,2:end) = vg_p_finger_data(:,2:end)/1000;

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

%% Grab the closest untracked points for each Image.
vg_aligned_finger_data = [];
for i = 1 : size(tango_img_timestamps, 1)
    [min_time_diff, min_idx] = min(abs(vg_p_finger_data(:,1) - tango_img_timestamps(i,2)));
    if min_time_diff > kMaxTimeDiffBetweenCamPoseAndFingerPosition 
        % If above threshold, zero this reading out.
        vg_aligned_finger_data = [vg_aligned_finger_data; 
                                  tango_img_timestamps(i,2), zeros(1,size(vg_p_finger_data, 2) - 1)];
        disp(['diff: ' num2str(min_time_diff)]);
        disp('continuing...');
    else 
        % Sync time with ba_cam_data.
        vg_aligned_finger_data = [vg_aligned_finger_data; 
                                  tango_img_timestamps(i,2), vg_p_finger_data(min_idx, 2:end)];
    end
end
   
%% Main loop through and display image with finger points.
if (DEBUG_IMAGES)
    fig2d = figure();
end
finger_boxes_data = [];
marker_pixel_data = [];
for i = start_image : min(size(tango_img_timestamps,1)-1, end_image)
    %%  Show the correct image.
    img_num = tango_img_timestamps(i,1);
    img_time = tango_img_timestamps(i,2);
    
    image_file = [image_dir 'image_' num2str(img_num, '%05d') '.pgm'];
    img = imread(image_file);   
    if DEBUG_IMAGES
        imshow(img);
        hold on;
    end

    disp(['timestamp: ' num2str(img_time)]);
    disp(['image num: ' num2str(img_num)]);
    
    %% Get positions of fingers.
    positions_data = vg_aligned_finger_data(i, 2:end);
    if (vg_aligned_finger_data(i,1) ~= img_time) 
        disp('ERROR syncing.');
        disp(['position time: ' num2str(positions_data(1), '%06f')]);
        disp(['tango time: ' num2str(img_time, '%06f')]);
        pause;
    end    
    
    
    %% Get cam_p_vg and cam_q_vg
    c_q_vg = quat_inv(quat_mul(vg_q_imu_(:,i), I_q_C));
    c_p_i = quat2rot(quat_inv(I_q_C)) * -I_p_C;
    i_p_vg = quat2rot(quat_inv(vg_q_imu_(:,i))) * -vg_p_imu_(:,i);
    c_p_vg = c_p_i + quat2rot(quat_inv(I_q_C)) * i_p_vg;
    
    inbounds_markers_matrix = [];
    inbounds_markers_vec = [];
    %% Check each marker at timestep and show if it is in image plane.
    for marker_i = 1 : 3 : (size(positions_data,2) - 2)
        vg_p_m = positions_data(marker_i : marker_i + 2)';
        if isempty(vg_p_m) 
            continue
        end

        if ~any(vg_p_m)
            % If all zeros, pad vector and continue.
            inbounds_markers_vec = [inbounds_markers_vec; 0;0];
            continue
        end
        c_ray_m = c_p_vg + quat2rot(c_q_vg) * vg_p_m; 
        if c_ray_m(3) < 0
            % If finger is behind camera, pad vector and continue.
            inbounds_markers_vec = [inbounds_markers_vec; 0;0];
            continue;
        end
        c_h_m = c_ray_m / c_ray_m(3);

        %% Convert to pixel coords
        pix = DistortRadial(c_h_m, fc, cc, kc);
%         pix = pixel_K * c_p_m;
%         pix = pix / pix(3);
        
        if pix(1) > 0 && pix(1) <= size(img,2) && pix(2) > 0 && pix(2) <= size(img,1)
            %% If on image, save it (and/or) show it.
            inbounds_markers_vec = [inbounds_markers_vec; pix];
            inbounds_markers_matrix = [inbounds_markers_matrix, pix];
            if (DEBUG_IMAGES)
                plot(pix(1), pix(2), '-c+');
            end
        else 
            %% else pad.
            inbounds_markers_vec = [inbounds_markers_vec; 0;0];
        end
    end
    
    %% Calculate the bounding box of finger in image.
    [top_left_corner, height, width] = GetBoundingBox(inbounds_markers_matrix, bounding_pixel_radius);
    top_left_corner = floor([min(max(top_left_corner(1), 0), size(img, 2));
                       min(max(top_left_corner(2), 0), size(img, 1))]);
    height = floor(min(height, size(img,1) - top_left_corner(2)));
    width = floor(min(width, size(img,2) - top_left_corner(1)));
    
    if DEBUG_IMAGES 
        rectangle('position', [top_left_corner', width, height]);
    end
    
    %% Save bounding box and marker pixels.
    finger_boxes_data = [finger_boxes_data;
                        img_num, top_left_corner', height, width];
    marker_pixel_data = [marker_pixel_data;
                        img_num, inbounds_markers_vec'];
   
    if (DEBUG_IMAGES) 
        disp('hit enter to continue...');
        pause;
        hold off;
    end
end

%% Save the bounding box data.
dlmwrite(box_output_file, finger_boxes_data, ',');
dlmwrite(marker_output_file, marker_pixel_data, ',');

disp('finished run');
close all;

