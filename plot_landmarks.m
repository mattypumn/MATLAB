close all; clear; clc;

%% Use MARS/matlab.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

%% Parameters.
data_dir= '/usr/local/google/home/mpoulter/for_matt/polaris/pr55_ws/45deg/';
height_file = '~/for_matt/height_out/stable_height.txt';
title_string = 'fast';
start_image = 6;
end_image = 600;
plot_range = [-10 10];

%%%%%%% Data files.      BE SURE TO SELECT THE PROPER FILE FORMAT
G_keypoints_dir = '~/Desktop/vio_points/';
% OR ...
% % data_path = '~/Desktop/global_points/';

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
height_data = dlmread(height_file);

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
for i = 1 : size(height_data,1)
    % Get the correct image number.
    timestamp_idx = find(abs(height_data(i,1) - timestamps(:,2)) < 1e-5);
    img_num = timestamps(timestamp_idx, 1);
    
    % Show for testing.
%     disp(['timestamp: ' num2str(tango_pose(i,1))]);
%     disp(['image data: ' num2str(img_num)]);
    
    % Calculate G_T_CL
%     G_p_I = tango_pose(i, 2:4)';
%     G_q_I = tango_pose(i,5:end)';
%     G_R_I = quat2rot(G_q_I);
%     G_T_I = [G_R_I, G_p_I;
%              0 0 0 1];
%     G_T_CL = G_T_I * I_T_CL;
    
    %%%%%%%%%%%% Read in the points from the Global frame.
    points_file = [G_keypoints_dir num2str(height_data(i,1), '%06f') '.txt'];
    if  exist(points_file, 'file')
        G_pts = dlmread(points_file);
    else 
        continue;
    end

    %%%%%%%%%%%%  OR: 
    
%     %%%%%%%%%%%% Read in the points from a camera point of view.
%     CL_pts = dlmread([data_path 'Cam_points_' num2str(img_num, '%06f') '.txt']);
%     % Homogenize and calculate transform.
%     point_count = size(CL_pts, 1);
%     G_pts = (G_T_CL * [CL_pts, ones(point_count,1)]')';

    % Plot.
    figure(fig3);
    plotted_data = plot3(G_pts(:,1), G_pts(:,2), G_pts(:,3), '.');
%     figure, plot(pts(:,1)./pts(:,3), -pts(:,2)./pts(:,3), '.')
    title([title_string '   image: ' num2str(img_num) '  ' ... 
             'timestamp: ' num2str(height_data(i,1), '%06f') ' ' ...
            'height: ' num2str(height_data(i, 2)) ... 
            ' point count: ' num2str(size(G_pts,1))]);
    plane = plot_plane(-height_data(i, 2));
    pause;
    delete(plotted_data);
    delete(plane);
end


function [plane] =  plot_plane(height) 
    pt1 = [-4, -4, height];
    pt2 = [-4, 4, height];
    pt3= [4, 4, height];
    pt4= [4, -4, height];
    points = [pt1', pt2', pt3', pt4'];
    plane = fill3(points(1,:), points(2,:), points(3,:), 'r');
    alpha(0.3);
end

% for i = 1 : size(tango_pose,1)
%     % Get the correct image number.
%     timestamp_idx = find(abs(tango_pose(i,1) - timestamps(:,2)) < 1e-5);
%     img_num = timestamps(timestamp_idx, 1);
%     height_idx = find(abs(tango_pose(i,1) - height_data(:,1)) < 1e-5);
%     
%     % Show for testing.
% %     disp(['timestamp: ' num2str(tango_pose(i,1))]);
% %     disp(['image data: ' num2str(img_num)]);
%     
%     % Calculate G_T_CL
%     G_p_I = tango_pose(i, 2:4)';
%     G_q_I = tango_pose(i,5:end)';
%     G_R_I = quat2rot(G_q_I);
%     G_T_I = [G_R_I, G_p_I;
%              0 0 0 1];
%     G_T_CL = G_T_I * I_T_CL;
%     
%     %%%%%%%%%%%% Read in the points from the Global frame.
%     points_file = [G_keypoints_dir num2str(tango_pose(i,1), '%06f') '.txt'];
%     if  exist(points_file, 'file')
%         G_pts = dlmread(points_file);
%     else 
%         continue;
%     end
% 
%     %%%%%%%%%%%%  OR: 
%     
% %     %%%%%%%%%%%% Read in the points from a camera point of view.
% %     CL_pts = dlmread([data_path 'Cam_points_' num2str(img_num, '%06f') '.txt']);
% %     % Homogenize and calculate transform.
% %     point_count = size(CL_pts, 1);
% %     G_pts = (G_T_CL * [CL_pts, ones(point_count,1)]')';
% 
%     % Plot.
%     figure(fig3);
%     plotted_data = plot3(G_pts(:,1), G_pts(:,2), G_pts(:,3), '.');
% %     figure, plot(pts(:,1)./pts(:,3), -pts(:,2)./pts(:,3), '.')
%     title([title_string '   image: ' num2str(img_num) '  ' ... 
%             num2str(tango_pose(i,1), '%06f') ' ' ...
%             'height: ' num2str(height_data(height_idx, 2))]);
%     pause;
%     delete(plotted_data);
% end



