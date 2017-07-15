close all; clear; clc;

%% Use MARS/matlab.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));



%% Parameters.
data_dir= '/usr/local/google/home/mpoulter/for_matt/polaris/pr55_ws/45deg/';

keypoints_dir_1 = '~/Desktop/global_points/';
legend_str_1 = 'mono w/ updated extrinsics';

% keypoints_dir_1 = '~/Desktop/pr5ws45_mono_3step_static-extrinsics/global_points/';
% legend_str_1 = 'mono w/o updated extrinsics';


% % PR55_ws_45 ground truth.
keypoints_dir_2 = '~/Desktop/prev_runs/pr55ws45_stereo_3step_updated-extrinsics/vio_points/';
legend_str_2 = 'stereo ground truth';

title_string = 'Mono and Stereo';
start_image = 37;
end_image = 600;
plot_range = [-10 10];

% %%%%%%% Data files.      BE SURE TO SELECT THE PROPER FILE FORMAT
% G_keypoints_dir = '~/Desktop/pr55_ws_45_stereo_points/fast_vio/vio_points/';
% OR ...
% data_path = '~/Desktop/global_points/';

%% Setup.
tango_pose_file = [data_dir '/out/tango_poses.txt'];
timestamp_file = [data_dir '/dump/feature_tracking_primary_timestamps.txt'];

% Extrinsics from calibration file.
I_p_CL = [0.002832067434431422;
          0.02677030293752162;
         -0.03023445403847945];
I_q_CL = [-0.9998932920483178;
           0.002607741735070337;
          -0.004576164107258534;
           0.01362581820971584];
I_R_CL = quat2rot(I_q_CL);
I_T_CL = [I_R_CL, I_p_CL; 0 0 0 1];



%% Parse Data Files.
timestamps = ReadImageTimestampFile(timestamp_file);
tango_pose = dlmread(tango_pose_file);

%% Grab only the timestamps within range.
timestamps = timestamps(timestamps(:,1) >= start_image & ...
                        timestamps(:,1) <= end_image, :);

% Grab the tango_poses correlated to the timestamps.
tango_bool  = zeros(size(tango_pose,1),1);
for i = 1 : size(tango_pose,1)
    if( any( abs(tango_pose(i,1) - timestamps(:,2)) < 1e-5) ) 
        tango_bool(i) = 1;
    end
end
tango_bool = logical(tango_bool);
tango_pose = tango_pose(tango_bool, :);

%% Build figure just once.
fig3 = figure();
xlabel('x'); ylabel('y'); zlabel('z');

xlim(plot_range);
ylim(plot_range);
zlim(plot_range);
%% Main Loop.
for i = 1 : size(tango_pose,1)
    % Get the correct image number.
    timestamp_idx = find(abs(tango_pose(i,1) - timestamps(:,2)) < 1e-5);
    img_num = timestamps(timestamp_idx, 1);
    
    % Show for testing.
    disp(['timestamp: ' num2str(tango_pose(i,1))]);
    disp(['image data: ' num2str(img_num)]);
    
    % Calculate G_T_CL
    G_p_I = tango_pose(i, 2:4)';
    G_q_I = tango_pose(i,5:end)';
    G_R_I = quat2rot(G_q_I);
    G_T_I = [G_R_I, G_p_I;
             0 0 0 1];
    G_T_CL = G_T_I * I_T_CL;
    
    G_pts_1 = [];
    %%%%%%%%%%%% Read in the points from the Global frame.
%     %%%%  For Mono-updated-extrinsics
%     points_file1 = [keypoints_dir_1 num2str(img_num) '.txt'];
%     %%%% For Mono persist extrinsics
    points_file1 = [keypoints_dir_1 'Cam_points_' num2str(img_num, '%06f') '.txt'];
    
    
    if  exist(points_file1, 'file')
        CL_pts = dlmread(points_file1);
        point_count = size(CL_pts, 1);
        G_pts_1 = (G_T_CL * [CL_pts, ones(point_count,1)]')';
    else 
        disp(['cant find points file for: ' num2str(img_num)]);
        pause;
        continue;
    end
    
    G_pts_2 = [];
    points_file2 = [keypoints_dir_2 num2str(tango_pose(i,1), '%06f') '.txt'];
    if  exist(points_file2, 'file')
        G_pts_2 = dlmread(points_file2);
    else 
        disp('cant find points_file2');
        continue;
    end

    disp(['mono features: ' num2str(size(G_pts_1, 1))]);
    disp(['stereo features: ' num2str(size(G_pts_2, 1))]);
    %%%%%%%%%%%%  OR: 
    
%     %%%%%%%%%%%% Read in the points from a camera point of view.
%     CL_pts = dlmread([data_path 'Cam_points_' num2str(img_num, '%06f') '.txt']);
%     % Homogenize and calculate transform.
%     point_count = size(CL_pts, 1);
%     G_pts = (G_T_CL * [CL_pts, ones(point_count,1)]')';

    % Plot.
%     figure(fig3);
%     plot3(G_pts_1(:,1), G_pts_1(:,2), G_pts_1(:,3), '.r');
    
    hold on;
    plot3(G_pts_2(:,1), G_pts_2(:,2), G_pts_2(:,3), '.b');
% %     figure, plot(pts(:,1)./pts(:,3), -pts(:,2)./pts(:,3), '.')
    title([title_string '   image: ' num2str(img_num)]);
%     legend(legend_str_1, legend_str_2);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    pause;
%     hold off;
end



