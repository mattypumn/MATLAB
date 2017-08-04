clear all; clc; close all;

mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));

out_path = '~/for_matt/height_out/';
data_path = '/usr/local/google/home/mpoulter/for_matt/polaris_floor_vicon/mrinal/4/';
vicon_cal_path = '/usr/local/google/home/mpoulter/for_matt/polaris_floor_vicon/mrinal/4/';

path_h = [out_path 'stable_height.txt'];
h_est = dlmread(path_h);

ext = dlmread([vicon_cal_path '/Tango_to_Vicon_Calibration.txt']);
v_p_i = ext(1,1:3)';
v_q_i = ext(2,:)';
i_p_c = [0.004718641446412659;  0.02439645089534794;  -0.02686148616790133] ;
i_q_c = [-0.9998604612232053;  0.003304862190576937;  -0.007085454619255722;  0.01476253031356873] ;
c_q_v = quat_mul(quat_inv(i_q_c), quat_inv(v_q_i));
c_p_v = quat2rot(i_q_c)' * (- i_p_c - quat2rot(v_q_i)' * v_p_i);

ts = dlmread([data_path '/time_alignment.txt']); ts = ts(2);
R = [0 0 1;1 0 0 ; 0 1 0];

vicon_data = dlmread([data_path 'target_poses.txt']);
v_valid = sum(vicon_data(:,2:end),2) ~= 0;
vicon_data = vicon_data(v_valid, :);
v_t = vicon_data(:,1)/500;
v_t = v_t + ts;
g_p_v = vicon_data(:,2:4)/1000;
v_q_g = vicon_data(:,5:8);
diffs = sum(diff(v_q_g),2);
sign_flag = 1;
for i=1:length(diffs)
    if abs(diffs(i))>0.1
        sign_flag = -sign_flag;
    end
    v_q_g(i+1,:) = v_q_g(i+1,:)*sign_flag;
end

for i=1:length(g_p_v)
    g_q_v(i,:) = quat_inv(v_q_g(i,:)')';
end

N = length(g_p_v);
g_p_c = zeros(3, N);
g_q_c = zeros(4, N);
for i=1:N
    g_p_c(:,i) = R * (g_p_v(i,:)' - quat2rot(g_q_v(i,:)') * quat2rot(c_q_v)' * c_p_v);
    g_q_c(:,i) = quat_mul(rot2quat(R), quat_mul(g_q_v(i,:)', quat_inv(c_q_v)));
end

path_traj = [data_path 'out/tango_poses.txt'];
traj = dlmread(path_traj);
t_traj = traj(:, 1);
gt_p_i = traj(:, 2:4);
gt_q_i = traj(:, 5:8);

cond = (t_traj <= max(v_t)) & (t_traj >= min(v_t)); 
t_traj = t_traj(cond);
gt_p_i = gt_p_i(cond,:);
gt_q_i = gt_q_i(cond,:);





%%

[g_p_c_, g_q_c_] = interp_poses(g_p_c, g_q_c, v_t, t_traj);

for i=1:length(t_traj)
    gt_p_c(:,i) = gt_p_i(i,:)' + quat2rot(gt_q_i(i,:)') * i_p_c;
    gt_q_c(:,i) = quat_mul(gt_q_i(i,:)', i_q_c);
end

[~,idx] = sort(h_est(:,1));
h_est = h_est(idx,:);

t_h = []; v_h = [];
time_h = [];
vicon_offset = 0.00; % .015
for i = 1:length(h_est)
    c_idx = find(abs(t_traj-h_est(i,1)) < 1e-5);
    if ~isempty(c_idx)
        time_h = [time_h h_est(i,1)];
        t_h = [t_h, h_est(i,2) + gt_p_c(3, c_idx)];
        v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
    end
end

% floor_counts = sum(t_h > 1.2)
% pause;

plot(time_h, t_h, 'b'), hold on, grid on
plot(time_h, v_h, 'r')
title('Floor height estimate')
legend('Tango', 'Vicon')
xlabel('time (s)'), ylabel('floor height (m)')
err = abs(v_h-t_h);
rmse_fh = sqrt(mean(err(t_h > 1).^2));

disp(['Floor height RMSE (m): ' num2str(rmse_fh)])