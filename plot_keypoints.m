close all; clear; clc;

%% Use MARS/matlab.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));



%% Parameters.
%%%%%%% Data files.      BE SURE TO SELECT THE PROPER FILE FORMAT
data_dir= '/usr/local/google/home/mpoulter/for_matt/polaris/pr55_ws/45deg/';
G_keypoints_dir = '~/Desktop/global_points/';
title_string = 'fast';
start_image = 1;
end_image = 600;
plot_range = [-20 20];

% OR ...1
% data_path = [data_dir 'dump/points/'];

%% Setup.
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
hold on;
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
    
    %%%%%%%%%%%% Read in the points from the Global frame.
    points_file = [G_keypoints_dir num2str(tango_pose(i,1), '%06f') '.txt'];
    if  exist(points_file, 'file')
        G_pts = dlmread(points_file);
    else 
        continue;
    end

    %%%%%%%%%%%%  OR: 
    
%     %%%%%%%%%%%% Read in the points from a camera point of view.
%     CL_pts = dlmread([data_path 'pts_' num2str(img_num) '.asc']);
%     % Homogenize and calculate transform.
%     point_count = size(CL_pts, 1);
%     G_pts = (G_T_CL * [CL_pts, ones(point_count,1)]')';

    % Plot.
    figure(fig3);
    plot3(G_pts(:,1), G_pts(:,2), G_pts(:,3), '.');
%     figure, plot(pts(:,1)./pts(:,3), -pts(:,2)./pts(:,3), '.')
    title([title_string '   image: ' num2str(img_num)]);
    pause;
end



