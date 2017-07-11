close all; clear; clc;

%% Parameters.
start_image = 568;
end_image = 821;
dataset = '~/for_matt/polaris/pr55_ws/45deg/';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vio_pose_file = [dataset 'out/tango_poses.txt'];
imu_pose_file = '/usr/local/google/home/mpoulter/Desktop/fast_vio/imu_pose.txt';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fast_points_dir = '~/for_matt/fast_points_pr55_kitchen_mrinalexp/';
% comp_points_dir = '~/for_matt/fast_points_pr55_kitchen_45_548pm/';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Read files.
vio_poses = dlmread(vio_pose_file);
imu_poses = dlmread(imu_pose_file);

t_start = min(min(vio_poses(:,1)), min(imu_poses(:,1)));
t_end = max(max(vio_poses(:,1)), max(imu_poses(:,1))) - t_start;

t_vio = vio_poses(:,1) - t_start;
t_imu = imu_poses(:,1) - t_start;

% 
% x_fig = figure();
% y_fig = figure();
% z_fig = figure();

% %% Plot Trajectory in 3D
fig_3d = figure();
disp(['tango/imu pose counts: ' num2str(size(vio_poses,1)) ' / ' num2str(size(imu_poses,1))]);
% set(fig, 'units','normalized','outerposition',[0 0 1 1]);
hold on;
plot3(imu_poses(:,2)', imu_poses(:,3)', imu_poses(:,4)', 'xr');
plot3(vio_poses(:,2)', vio_poses(:,3)', vio_poses(:,4)', '+b');

legend('imu poses', 'vio poses');


%% Plot RPY
for i = 1 : size(vio_poses,1)
    vio_rpy(:,i) = rot2rpy(quat2rot(vio_poses(i,5:end)'));
end
for i = 1 : size(imu_poses,1)
    imu_rpy(:,i) = rot2rpy(quat2rot(imu_poses(i,5:end)'));
end

% update IMU plot.
rpyFig = figure();

subplot(3,1,1); plot(t_imu, imu_rpy(3,:), 'xr'); title('roll');
hold on;

subplot(3,1,2); plot(t_imu, imu_rpy(1,:), 'xr'); title('pitch');
hold on;
subplot(3,1,3); plot(t_imu, imu_rpy(2,:), 'xr'); title('yaw');
hold on;

subplot(3,1,1); plot(t_vio, vio_rpy(3,:), '+b'); title('roll');
subplot(3,1,2); plot(t_vio, vio_rpy(1,:), '+b'); title('pitch');
subplot(3,1,3); plot(t_vio, vio_rpy(2,:), '+b'); title('yaw');
legend('imu poses', 'vio poses');


norm_diff = [];
for i = 1 : size(vio_poses, 1) 
    pose_t = imu_poses(vio_poses(i,1) == imu_poses(:,1), :);
    if ~isempty(pose_t)
    norm_diff = [norm_diff; norm(pose_t(2:4) - vio_poses(i,2:4)), norm(pose_t(5:8) - vio_poses(i,5:8))];
    end
end

plot(norm_diff(:,1)), figure, plot(norm_diff(:,2))


