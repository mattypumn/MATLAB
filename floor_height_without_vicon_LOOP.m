clear all; clc; close all;

mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));


height_thresh = 0.8;
data_loc = {'_pr55_hall_', '_pr55_kitchen_', '_pr55_patio_', ... 
            '_sb55_ws_','_sb10_juice_', '_sb10_meditation_', ...
            '_sb65_hall_', '_sb65_kitchen_', '_sb65_office_', ...
            '_sb65_office_', '_sb65_road_'};
angles = {'0', '45', '50', '90'};


height_path = '~/for_matt/height_out/';
% height_path = '';

data_path_FASTKLT = '~/for_matt/height_out/stable_height_fixing-fast-branch_FAST-KLT/';
data_path_NONE = '~/for_matt/height_out/stable_height_fixing-fast-branch_NONE/';
file_base = 'stable_height';
file_ext = '.txt';


for loc = 1 : size(data_loc,2)
    for  ang = 1 : size(angles, 2)
        loc_str = data_loc{loc};
        ang_str = angles{ang};
        data_path = ['~/for_matt/polaris/' loc_str(2:end-1) '/' ang_str 'deg/'];
        path_traj = [data_path 'out/tango_poses.txt'];
        fk_file = [data_path_FASTKLT file_base loc_str ang_str file_ext];
        h_est_fk = [];
        h_est_none = [];
        if ( exist(fk_file, 'file') ~= 2)
            disp(['Unable to read file: ' fk_file]); 
            continue;
        else 
            h_est_fk = dlmread(fk_file); 
        end
        none_file = [data_path_NONE file_base loc_str ang_str file_ext];
        if ( exist(none_file,'file') ~= 2 )
            disp(['Unable to read file: ' none_file]);
        else
            h_est_none = dlmread(none_file);  
        end
        
        % 

        i_p_c = [0.004718641446412659;  0.02439645089534794;  -0.02686148616790133] ;
        i_q_c = [-0.9998604612232053;  0.003304862190576937;  -0.007085454619255722;  0.01476253031356873] ;

        R = [0 0 1;1 0 0 ; 0 1 0];


        traj = dlmread(path_traj);
        t_traj = traj(:, 1);
        gt_p_i = traj(:, 2:4);
        gt_q_i = traj(:, 5:8);


        for i=1:length(t_traj)
            gt_p_c(:,i) = gt_p_i(i,:)' + quat2rot(gt_q_i(i,:)') * i_p_c;
            gt_q_c(:,i) = quat_mul(gt_q_i(i,:)', i_q_c);
        end

        [~,idx] = sort(h_est_fk(:,1));
        h_est_fk = h_est_fk(idx,:);



        t_h_w = []; v_h = [];
        time_h_w = [];
        % vicon_offset = 0.00; % .015
        for i = 1:length(h_est_fk)
            c_idx = find(abs(t_traj-h_est_fk(i,1)) < 1e-5);
            if ~isempty(c_idx)
                time_h_w = [time_h_w h_est_fk(i,1)];
                t_h_w = [t_h_w, h_est_fk(i,2) + gt_p_c(3, c_idx)];
        %         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
            end
        end

        t_h_wo = []; v_h = [];
        time_h_wo = [];
        if ~isempty(h_est_none)
            [~,idx_wo] = sort(h_est_none(:,1));
            h_est_none = h_est_none(idx_wo,:);
            
            % vicon_offset = 0.00; % .015
            for i = 1:length(h_est_none)
                c_idx = find(abs(t_traj-h_est_none(i,1)) < 1e-5);
                if ~isempty(c_idx)
                    time_h_wo = [time_h_wo h_est_none(i,1)];
                    t_h_wo = [t_h_wo, h_est_none(i,2) + gt_p_c(3, c_idx)];
            %         v_h = [v_h, g_p_c_(3, c_idx) + vicon_offset];
                end
            end
        end
        % disp(['floor count with: ' num2str(sum(t_h_w > height_thresh))]);
        % disp(['floor count without: ' num2str(sum(t_h_wo > height_thresh))]);

        fig = figure('units','normalized','outerposition',[0 0 1 1]);
        % fig = figure();

        title_str = ['FAST-KLT Heights (' loc_str(2:5) ' ' loc_str(7:end-1) ' ' ang_str 'degrees)'];

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

        img_out_str = [loc_str(2:end) ang_str '.jpg'];
        print(fig, '-djpeg', '-r0', img_out_str);
        
        close all;
    end
end









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