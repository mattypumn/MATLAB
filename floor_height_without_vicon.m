clear all; clc; close all;

%% Using MARS Matlab.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

%% Parameters. 
data = 'pr55_hall_0';
output_heights_none = '~/for_matt/height_out/updated_calib_heights/none/';
output_heights_fklt = '~/for_matt/height_out/updated_calib_heights/fklt/';

%% Data set-up.
height_thresh = 1.2;
underscore_idx = find(data== '_');
% Tango output.
data_path = ['~/for_matt/polaris/' data(1:underscore_idx(2)-1) '/' ...
              data(underscore_idx(2)+1:end) 'deg/'];
path_traj = [data_path 'out/tango_poses.txt'];
timestamp_file = [data_path 'dump/feature_tracking_primary_timestamps.txt'];

% Printed heights.
file_FASTKLT = [output_heights_fklt 'stable_height_' data '_fklt.txt'];
file_NONE = [output_heights_none 'stable_height_' data '_none.txt'];

h_est_fk = [];
h_est_none = [];
if ( exist(file_FASTKLT, 'file') ~= 2)
    disp(['Unable to read file: ' file_FASTKLT]);
    % User must assert missing file.
    pause;
else
    h_est_fk = dlmread(file_FASTKLT);
end

if ( exist(file_NONE,'file') ~= 2 )
    disp(['Unable to read file: ' file_NONE]);
    % User must assert missing file.
    pause;
else
    h_est_none = dlmread(file_NONE);
end
%% Camera Extrinsics.
i_p_c = [0.004718641446412659;  0.02439645089534794;  -0.02686148616790133] ;
i_q_c = [-0.9998604612232053;  0.003304862190576937;  -0.007085454619255722;  0.01476253031356873] ;
R = [0 0 1;1 0 0 ; 0 1 0];

%% Read in Trajectory And convert to global reference frame.
traj = dlmread(path_traj);
t_traj = traj(:, 1);
gt_p_i = traj(:, 2:4);
gt_q_i = traj(:, 5:8);

for i=1:length(t_traj)
    gt_p_c(:,i) = gt_p_i(i,:)' + quat2rot(gt_q_i(i,:)') * i_p_c;
    gt_q_c(:,i) = quat_mul(gt_q_i(i,:)', i_q_c);
end
%  Sort all values by timestamps.
[~,idx] = sort(h_est_fk(:,1));
h_est_fk = h_est_fk(idx,:);
if ~isempty(h_est_none)
    [~,idx_none] = sort(h_est_none(:,1));
    h_est_none = h_est_none(idx_none,:);
end

%% Read in timestamps to calculate t = 0.
timestamps = ReadImageTimestampFile(timestamp_file);
start_time = min(timestamps(:,2));
final_time = max(timestamps(:,2));


%% Average the heights from update within the tango time interval.
g_h_fk = []; %v_h = [];info_area
time_h_fk = [];
g_h_none = []; %v_h = [];
time_h_none = [];
% vicon_offset = 0.00; % .015
for time = 1 : length(t_traj)
    if time ==1
        %% TODO Get start of data timestamp.
        t_start = start_time;
        continue;
    else
        t_start = t_traj(time - 1);
    end
    t_end = t_traj(time);
    
    c_idx_fk = find(h_est_fk(:,1) > t_start & ...
                    h_est_fk(:,1) <= t_end);
    tmp_h_est = mean(h_est_fk(c_idx_fk, 2));
    if (~isempty(tmp_h_est) && ~isnan(tmp_h_est))
        time_h_fk = [time_h_fk, t_end];
        g_h_fk = [g_h_fk, tmp_h_est + gt_p_c(3, time)];
    end
    
    if isempty(h_est_none) 
        continue;
    end
    c_idx_none = find(h_est_none(:,1) > t_start & ...
                      h_est_none(:,1) <= t_end);
    tmp_h_est = mean(h_est_none(c_idx_none, 2));
    if ~isempty(tmp_h_est) && ~isnan(tmp_h_est)
        time_h_none = [time_h_none t_end];
        g_h_none = [g_h_none, tmp_h_est + gt_p_c(3, time)];
    end
end


%%  Use only the heights corresponding to tango time interval. 
% for i = 1:length(h_est_fk)
%     C_h_estimate = h_est_fk(i,:);
%     c_idx = find(abs(t_traj-h_est_fk(i,1)) < 1e-5);
%     if ~isempty(c_idx)
%         time_h_w = [time_h_w h_est_fk(i,1)];
%         t_h_w = [t_h_w, h_est_fk(i,2) + gt_p_c(3, c_idx)];
% %         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
%     end
% end
% 
% t_h_wo = []; %v_h = [];
% time_h_wo = [];
% if ~isempty(h_est_none)
%     [~,idx_wo] = sort(h_est_none(:,1));
%     h_est_none = h_est_none(idx_wo,:);
% 
%     % vicon_offset = 0.00; % .015
%     for i = 1:length(h_est_none)
%         c_idx = find(abs(t_traj-h_est_none(i,1)) < 1e-5);
%         if ~isempty(c_idx)
%             time_h_wo = [time_h_wo h_est_none(i,1)];
%             t_h_wo = [t_h_wo, h_est_none(i,2) + gt_p_c(3, c_idx)];
%     %         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
%         end
%     end
% end
% disp(['floor count with: ' num2str(sum(t_h_w > height_thresh))]);
% disp(['floor count without: ' num2str(sum(t_h_wo > height_thresh))]);

%% Plot.
% fig = figure('units','normalized','outerposition',[0 0 1 1]);
% fig = figure();
fig = figure('units','normalized','outerposition',[0 0 .5 .5]);

room_str = data(underscore_idx(1)+1 : underscore_idx(2)-1);
deg_str = data(underscore_idx(2)+1:end);
title_str = ['Floor Heights [Dataset: ' data(1:4) '-' room_str ' @ ' deg_str sprintf('%c', char(176)) ']'];



% axes('Position',[.25 .1 .7 .7])
% 
time_h_fk = time_h_fk - start_time;
time_h_none = time_h_none - start_time;
plot(time_h_fk, g_h_fk, 'xb');
hold on;
plot(time_h_none, g_h_none, 'or');
hold on;
% plot(time_h_w, height_thresh, '+y');
xlim([0, (final_time-start_time)]);
ylim([0,2]);
title(title_str);
ylabel('Height in Camera (m)');
xlabel('Time (s)');
legend('FAST-KLT','VIO inliers');

info_area = axes('Position',[.15 -.3 1 1],'Visible','off');
% info_str(1) = {sprintf('Total Poses:  %d', size(t_traj, 1))};
info_str(1) = {sprintf('Estimates w/ FAST-KLT :  %d', sum(g_h_fk > height_thresh))};
info_str(2) = {sprintf('Estimates w/ VIO inliers:  %d', sum(g_h_none > height_thresh))};
info_str(3) = {sprintf('Height Threshold: %0.1f m', height_thresh)};


set(gcf,'CurrentAxes',info_area);
text(.025,.6,info_str,'FontSize',12);

%% Save Image.
img_out_str = [data '.jpg'];
print(fig, '-djpeg', '-r0', img_out_str);





%% Junk stuff.


% module_file = sprintf('%sstable_height_data%d_with-module.txt', height_path, ds);
% no_module_file = sprintf('%sstable_height_data%d_no-module.txt', height_path, ds);

% path_h = [height_path 'stable_height_data2_with-module.txt'];
% path_h = module_file;
% path_h = no_module_file;

% 
% h_est_with = dlmread(module_file);
% h_est_without = dlmread(no_module_file);
% 
% i_p_c = [0.004718641446412659;  0.02439645089534794;  -0.02686148616790133] ;
% i_q_c = [-0.9998604612232053;  0.003304862190576937;  -0.007085454619255722;  0.01476253031356873] ;
% 
% R = [0 0 1;1 0 0 ; 0 1 0];
% 
% 
% path_traj = [data_path 'out/tango_poses.txt'];
% traj = dlmread(path_traj);
% t_traj = traj(:, 1);
% gt_p_i = traj(:, 2:4);
% gt_q_i = traj(:, 5:8);
% 
% 
% for i=1:length(t_traj)
%     gt_p_c(:,i) = gt_p_i(i,:)' + quat2rot(gt_q_i(i,:)') * i_p_c;
%     gt_q_c(:,i) = quat_mul(gt_q_i(i,:)', i_q_c);
% end
% 
% [~,idx] = sort(h_est_with(:,1));
% h_est_with = h_est_with(idx,:);
% 
% [~,idx_wo] = sort(h_est_without(:,1));
% h_est_without = h_est_without(idx_wo,:);
% 
% t_h_w = []; v_h = [];
% time_h_w = [];
% % vicon_offset = 0.00; % .015
% for i = 1:length(h_est_with)
%     c_idx = find(abs(t_traj-h_est_with(i,1)) < 1e-5);
%     if ~isempty(c_idx)
%         time_h_w = [time_h_w h_est_with(i,1)];
%         t_h_w = [t_h_w, h_est_with(i,2) + gt_p_c(3, c_idx)];
% %         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
%     end
% end
% 
% t_h_wo = []; v_h = [];
% time_h_wo = [];
% % vicon_offset = 0.00; % .015
% for i = 1:length(h_est_without)
%     c_idx = find(abs(t_traj-h_est_without(i,1)) < 1e-5);
%     if ~isempty(c_idx)
%         time_h_wo = [time_h_wo h_est_without(i,1)];
%         t_h_wo = [t_h_wo, h_est_without(i,2) + gt_p_c(3, c_idx)];
% %         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
%     end
% end
% 
% 
% % disp(['floor count with: ' num2str(sum(t_h_w > height_thresh))]);
% % disp(['floor count without: ' num2str(sum(t_h_wo > height_thresh))]);
% 
% fig = figure('units','normalized','outerposition',[0 0 1 1]);
% % fig = figure();
% 
% title_str = sprintf('Heights with and without module data%d', ds);
% 
% info_area = axes('Position',[0 0 1 1],'Visible','off');
% axes('Position',[.25 .1 .7 .7])
% % 
% % figure();
% plot(time_h_w, t_h_w, 'xb');
% hold on;
% plot(time_h_wo, t_h_wo, '+r');
% hold on;
% % plot(time_h_w, height_thresh, '+y');
% 
% title(title_str);
% ylabel('Height (m)');
% xlabel('timesamp');
% legend('with module','without module');
% 
% 
% info_str(1) = {sprintf('height threshold (m):  %0.3f', height_thresh)};
% info_str(2) = {sprintf('with module count:  %d', sum(t_h_w > height_thresh))};
% info_str(3) = {sprintf('without module count:  %d', sum(t_h_wo > height_thresh))};
% 
% 
% set(gcf,'CurrentAxes',info_area);
% text(.025,.6,info_str,'FontSize',12);
% 
% img_out = sprintf('data%d.jpg', ds);
% print(fig, '-djpeg', '-r0', img_out);
% saveas(fig, img_out,'auto');





% pause;
% 
% plot(time_h, t_h, 'b'), hold on, grid on
% plot(time_h, v_h, 'r')
% title('Floor height estimate')
% legend('Tango', 'Vicon')
% xlabel('time (s)'), ylabel('floor height (m)')
% err = abs(v_h-t_h);
% rmse_fh = sqrt(mean(err(t_h > 1).^2));
% 
% disp(['Floor height RMSE (m): ' num2str(rmse_fh)])