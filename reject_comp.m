clear; close all; clc;


%% Parameters.
data_dir = '~/for_matt/polaris/pr55_ws/45deg/';
rej_dir = '~/Desktop/rejection_log/';
start_image = 27;
end_image = 500;
step_size = 3;

%% Setup.
im_dir = [data_dir 'dump/feature_tracking_primary/'];
tango_pose_file = [data_dir '/out/tango_poses.txt'];
timestamp_file = [data_dir '/dump/feature_tracking_primary_timestamps.txt'];

% Extrinsics from calibration file.
I_p_CL = [0.001690537285138292;
          0.02689805347682736;
         -0.02104682309162398];
I_q_CL = [-0.9998612388900842;
           0.003806400931241;
          -0.009040478629248507;
           0.01346417555184306];
I_R_CL = quat2rot(I_q_CL);
I_T_CL = [I_R_CL, I_p_CL; 0 0 0 1];

%% Load Timestamp data.
timestamps = ReadImageTimestampFile(timestamp_file);
timestamps = timestamps( timestamps(:,1) >= start_image & ...
                         timestamps(:,1) <= end_image);


%% Loop over each image.
figure();
for i = start_image:step_size:end_image
    %% Read the image.
    im_file = [im_dir 'image_' num2str(i, '%05d') '.pgm'];
    im = imread(im_file);
    %% Read in the reject info.
    % px_l py_l px_r py_r epipolar depth reprojection
    rej_file = [rej_dir num2str(i, '%06f') '.txt'];
    rej_data = dlmread(rej_file);
    
    %% Extract points to plot.
    epipolar_rejects = rej_data((rej_data(:,5) == 1), 1:2);
    depth_rejects = rej_data((rej_data(:,6) == 1), 1:2);
    reproj_rejects = rej_data((rej_data(:,7) == 1), 1:2);
    
    
    %% Plot.
    imshow(im);
    hold on;
    title('outlier rejection');
    plot(epipolar_rejects(:,1)', epipolar_rejects(:,2)', '+r');
    plot(depth_rejects(:,1)', depth_rejects(:,2)', 'xb');
    plot(reproj_rejects(:,1)', reproj_rejects(:,2)', 'og');
    legend('epipolar rejects', ... 
           'depth rejects', ...
           'reprojection rejects');
    
    
    hold off;
    
    pause();
end