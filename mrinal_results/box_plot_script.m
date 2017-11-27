close all; clear; clc;

%% Params.
file = 'data.mat';

%% Load all of the data
data = load(file);

pipeline_labels = data.pipeline_labels;
new_lengths = data.new_lengths;
euroc_lengths = data.euroc_lengths;
mars_mono_15hz_euroc = data.mars_mono_15hz_euroc;
mars_mono_30hz_euroc = data.mars_mono_30hz_euroc;
mars_mono_15hz_new = data.mars_mono_15hz_new;
mars_mono_30hz_new = data.mars_mono_30hz_new;

mars_stereo_15hz_euroc = data.mars_stereo_15hz_euroc;
mars_stereo_30hz_euroc = data.mars_stereo_30hz_euroc;
mars_stereo_15hz_new = data.mars_stereo_15hz_new;
mars_stereo_30hz_new = data.mars_stereo_30hz_new;

alt_stereo_30hz_euroc = data.alt_stereo_30hz_euroc;
mono_30hz_m20_euroc = data.mono_30hz_m20_euroc;
stereo_30hz_m20_euroc = data.stereo_30hz_m20_euroc;
alt_stereo_30hz_new = data.alt_stereo_30hz_new;
mono_30hz_m20_new = data.mono_30hz_m20_new;
stereo_30hz_m20_new = data.stereo_30hz_m20_new;

%%  Data prep.
%%%%%%%%  Spreadsheet ordering.
% euroc_mat = [mars_mono_15hz_euroc, mars_mono_30hz_euroc, ...
%              mars_stereo_15hz_euroc, mars_stereo_30hz_euroc, ...
%              alt_stereo_30hz_euroc, mono_30hz_m20_euroc, ...
%              stereo_30hz_m20_euroc];
% new_mat = [mars_mono_15hz_new, mars_mono_30hz_new, ...
%              mars_stereo_15hz_new, mars_stereo_30hz_new, ...
%              alt_stereo_30hz_new, mono_30hz_m20_new, ...
%              stereo_30hz_m20_new];

%%%%%%%%%%%%  [mono stereo]
euroc_mat = [mars_mono_15hz_euroc, ...
             mars_mono_30hz_euroc, ...
             mono_30hz_m20_euroc, ...
             alt_stereo_30hz_euroc, ...
             mars_stereo_15hz_euroc, ...
             mars_stereo_30hz_euroc,...
             stereo_30hz_m20_euroc];
new_mat = [mars_mono_15hz_new, ...
           mars_mono_30hz_new, ...
           mono_30hz_m20_new, ...
           alt_stereo_30hz_new, ...
           mars_stereo_15hz_new, ...
           mars_stereo_30hz_new, ...
           stereo_30hz_m20_new];


%% Plot
% euroc.
% figure()
% hold on;
% boxplot(euroc_mat, 'labels', pipeline_labels)
% title('euroc');
% ylabel('RMSE');
% xlabel('Pipelines');
% hold off;
% pause;
% 
% % euroc normalized.
% figure()
% hold on;
% boxplot((euroc_mat ./ euroc_lengths) * 100, 'labels', pipeline_labels)
% title('euroc normalized');
% ylabel('RMSE (%)');
% xlabel('Pipelines');
% hold off;
% pause;
% 
% 
% 
% % mars.
% figure()
% hold on;
% boxplot(new_mat, 'labels', pipeline_labels)
% title('mars');
% ylabel('RMSE');
% xlabel('Pipelines');
% pause(0.01);
% hold off;
% pause;
% 
% % mars normalized.
% figure()
% hold on;
% boxplot(new_mat ./ new_lengths * 100, 'labels', pipeline_labels)
% title('mars normalized');
% ylabel('RMSE (%)');
% xlabel('Pipelines');
% hold off;
% pause;

%%  Prepped data and saved.
% save(file, 'mars_stereo_15hz_euroc', 'mars_stereo_30hz_euroc', ...
%                 'mars_stereo_15hz_new', 'mars_stereo_30hz_new', ...
%                 'alt_stereo_30hz_euroc', 'mono_30hz_m20_euroc', ...
%                 'stereo_30hz_m20_euroc', 'mars_mono_15hz_euroc', ...
%                 'mars_mono_30hz_euroc', 'mars_mono_15hz_new',...
%                 'alt_stereo_30hz_new', 'mono_30hz_m20_new', ...
%                 'stereo_30hz_m20_new', 'mars_mono_30hz_new', ...
%                 'pipeline_labels', 'new_lengths', 'euroc_lengths');

% 
% data(:,1:7) = (data(:,1:7) ./ repmat(len(1:7), 2, 1) * 100);
% rmse = data';



% new_padded_mat = NaN(size(euroc_mat));
% new_padded_mat(1:size(new_mat, 1), 1:size(new_mat, 2)) = new_mat;
% all_data = [euroc_mat, new_padded_mat];
%% Prep the data.
all_data_rmse = NaN(size(euroc_mat) + [0, size(new_mat, 2)]);
all_data_rmse(:,1) = euroc_mat(:,1);
all_data_rmse(1:6,2) = new_mat(:,1);
all_data_rmse(:,3) = euroc_mat(:,2);
all_data_rmse(1:6,4) = new_mat(:,2);
all_data_rmse(:,5) = euroc_mat(:,3);
all_data_rmse(1:6,6) = new_mat(:,3);
all_data_rmse(:,7) = euroc_mat(:,4);
all_data_rmse(1:6,8) = new_mat(:,4);
all_data_rmse(:,9) = euroc_mat(:,5);
all_data_rmse(1:6,10) = new_mat(:,5);
all_data_rmse(:,11) = euroc_mat(:,6);
all_data_rmse(1:6,12) = new_mat(:,6);
all_data_rmse(:,13) = euroc_mat(:,7);
all_data_rmse(1:6,14) = new_mat(:,7);


all_data_normalized = NaN(size(euroc_mat) + [0, size(new_mat, 2)]);
euroc_norm_mat = euroc_mat ./ euroc_lengths*100;
new_norm_mat = new_mat ./ new_lengths*100;
all_data_normalized(:,1) = euroc_norm_mat(:,1);
all_data_normalized(1:6,2) = new_norm_mat(:,1);
all_data_normalized(:,3) = euroc_norm_mat(:,2);
all_data_normalized(1:6,4) = new_norm_mat(:,2);
all_data_normalized(:,5) = euroc_norm_mat(:,3);
all_data_normalized(1:6,6) = new_norm_mat(:,3);
all_data_normalized(:,7) = euroc_norm_mat(:,4);
all_data_normalized(1:6,8) = new_norm_mat(:,4);
all_data_normalized(:,9) = euroc_norm_mat(:,5);
all_data_normalized(1:6,10) = new_norm_mat(:,5);
all_data_normalized(:,11) = euroc_norm_mat(:,6);
all_data_normalized(1:6,12) = new_norm_mat(:,6);
all_data_normalized(:,13) = euroc_norm_mat(:,7);
all_data_normalized(1:6,14) = new_norm_mat(:,7);

% all_data_cell{} = {};
% all_data_cell{1} = euroc_mat(:,1);
% all_data_cell{2} = new_mat(:,1);
% all_data_cell{3} = euroc_mat(:,2);
% all_data_cell{4} = new_mat(:,2);
% all_data_cell{5} = euroc_mat(:,3);
% all_data_cell{6} = new_mat(:,3);
% all_data_cell{7} = euroc_mat(:,4);
% all_data_cell{8} = new_mat(:,4);
% all_data_cell{9} = euroc_mat(:,5);
% all_data_cell{10} = new_mat(:,5);
% all_data_cell{11} = euroc_mat(:,6);
% all_data_cell{12} = new_mat(:,6);
% all_data_cell{13} = euroc_mat(:,7);
% all_data_cell{14} = new_mat(:,7);

tag = [1 2 3 4 5 6 7]';


separation = 0.15;
offset = separation / 2;
% 
% positions = [1 1.25 ...
%             1.75 2 ...
%             2.5 2.75 ...
%             3.25 3.5 ...
%             4 4.25 ...
%             4.75 5 ...
%             5.5 5.75 ];
        
xtick = [1.125, 1.875, 2.625, 3.375, 4.125, 4.875, 5.625];
positions = xtick';%[xtick'-offset, xtick'+offset];
positions = reshape(positions', 1, []);


%% Plot 
figure
%%%%%%%%%%%%%%%%%%%%
boxplot(all_data_rmse(:,1:2:end), tag, 'positions', positions, 'widths', 0.07);
xlim([0.8,6]);
% ylim([0 0.9]);
% ylim([0 2]);
pipeline_labels = ['','','','','','',''];
set(gca,'xtick', xtick);
set(gca,'xticklabel', ...
        pipeline_labels )
% color = ['b', 'b', 'b', 'b', 'b', 'b', 'b'];
% h = findobj(gca,'Tag','Box');
% for j=1:length(h)
%    patch(get(h(j),'XData'),get(h(j),'YData'),color(j),'FaceAlpha',.25);
% end
% c = get(gca, 'Children');
% hleg1 = legend(c(1:2), '30 Hz datasets', '25 Hz datasets');
ylabel('RMSE (m)');

%% Plot 
figure
%%%%%%%%%%%%%%%%%%%%
boxplot(all_data_rmse(1:6,2:2:end), tag, 'positions', positions, 'widths', 0.07);
xlim([0.8,6]);
% ylim([0 0.9]);
% ylim([0 2]);
set(gca,'xtick', xtick);
set(gca,'xticklabel', ...
        pipeline_labels )
% color = ['b', 'b', 'b', 'b', 'b', 'b', 'b'];
% h = findobj(gca,'Tag','Box');
% for j=1:length(h)
%    patch(get(h(j),'XData'),get(h(j),'YData'),color(j),'FaceAlpha',.25);
% end
% c = get(gca, 'Children');
% hleg1 = legend(c(1:2), '30 Hz datasets', '25 Hz datasets');
ylabel('RMSE (m)');

%%%%%%%%%%%%%  Plot normalization.
% figure()
% boxplot(all_data_normalized, tag, 'positions', positions, 'widths', 0.07);
% % ylim([0 2]);
% set(gca,'xtick',xtick)
% set(gca,'xticklabel', ...
%         pipeline_labels )
% color = ['r', 'b', 'r', 'b', 'r', 'b', 'r', 'b', 'r', 'b', 'r', 'b', 'r', 'b'];
% h = findobj(gca,'Tag','Box');
% for j=1:length(h)
%    patch(get(h(j),'XData'),get(h(j),'YData'),color(j),'FaceAlpha',.5);
% end
% c = get(gca, 'Children');
% hleg1 = legend(c(1:2), '30 Hz', '25 Hz');
% ylabel('RMSE (%)');

%             
            
            