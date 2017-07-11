clear all; clc; close all;

mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

ds = 3;
height_thresh = 0.8;



% height_path = '~/for_matt/height_out/';
height_path = '';

data_path = sprintf('~/for_matt/bags_only/data%d/', ds);

module_file = [height_path 'stable_height_with-module.txt'];
if ( exist(module_file, 'file') ~= 2)
    disp(['Unable to read file: ' module_file]); 
    quit;
end
no_module_file = [height_path 'stable_height_no-module.txt'];

if ( exist(no_module_file,'file') ~= 2 )
    disp(['Unable to read file: ' no_module_file]);
    quit;
end



% module_file = sprintf('%sstable_height_data%d_with-module.txt', height_path, ds);
% no_module_file = sprintf('%sstable_height_data%d_no-module.txt', height_path, ds);

% path_h = [height_path 'stable_height_data2_with-module.txt'];
% path_h = module_file;
% path_h = no_module_file;


h_est_with = dlmread(module_file);
h_est_without = dlmread(no_module_file);

i_p_c = [0.004718641446412659;  0.02439645089534794;  -0.02686148616790133] ;
i_q_c = [-0.9998604612232053;  0.003304862190576937;  -0.007085454619255722;  0.01476253031356873] ;

R = [0 0 1;1 0 0 ; 0 1 0];


path_traj = [data_path 'out/tango_poses.txt'];
traj = dlmread(path_traj);
t_traj = traj(:, 1);
gt_p_i = traj(:, 2:4);
gt_q_i = traj(:, 5:8);


for i=1:length(t_traj)
    gt_p_c(:,i) = gt_p_i(i,:)' + quat2rot(gt_q_i(i,:)') * i_p_c;
    gt_q_c(:,i) = quat_mul(gt_q_i(i,:)', i_q_c);
end

[~,idx] = sort(h_est_with(:,1));
h_est_with = h_est_with(idx,:);

[~,idx_wo] = sort(h_est_without(:,1));
h_est_without = h_est_without(idx_wo,:);

t_h_w = []; v_h = [];
time_h_w = [];
% vicon_offset = 0.00; % .015
for i = 1:length(h_est_with)
    c_idx = find(abs(t_traj-h_est_with(i,1)) < 1e-5);
    if ~isempty(c_idx)
        time_h_w = [time_h_w h_est_with(i,1)];
        t_h_w = [t_h_w, h_est_with(i,2) + gt_p_c(3, c_idx)];
%         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
    end
end

t_h_wo = []; v_h = [];
time_h_wo = [];
% vicon_offset = 0.00; % .015
for i = 1:length(h_est_without)
    c_idx = find(abs(t_traj-h_est_without(i,1)) < 1e-5);
    if ~isempty(c_idx)
        time_h_wo = [time_h_wo h_est_without(i,1)];
        t_h_wo = [t_h_wo, h_est_without(i,2) + gt_p_c(3, c_idx)];
%         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
    end
end


% disp(['floor count with: ' num2str(sum(t_h_w > height_thresh))]);
% disp(['floor count without: ' num2str(sum(t_h_wo > height_thresh))]);

fig = figure('units','normalized','outerposition',[0 0 1 1]);
% fig = figure();

title_str = sprintf('Heights with and without module data%d', ds);

info_area = axes('Position',[0 0 1 1],'Visible','off');
axes('Position',[.25 .1 .7 .7])
% 
% figure();
plot(time_h_w, t_h_w, 'xb');
hold on;
plot(time_h_wo, t_h_wo, '+r');
hold on;
% plot(time_h_w, height_thresh, '+y');

title(title_str);
ylabel('Height (m)');
xlabel('timesamp');
legend('with module','without module');


info_str(1) = {sprintf('height threshold (m):  %0.3f', height_thresh)};
info_str(2) = {sprintf('with module count:  %d', sum(t_h_w > height_thresh))};
info_str(3) = {sprintf('without module count:  %d', sum(t_h_wo > height_thresh))};


set(gcf,'CurrentAxes',info_area);
text(.025,.6,info_str,'FontSize',12);

img_out = 'comp.jpg';
print(fig, '-djpeg', '-r0', img_out);
% saveas(fig, img_out,'auto');


quit;

